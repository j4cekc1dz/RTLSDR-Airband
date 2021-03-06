/*
 * mixer.cpp
 * Mixer related routines
 *
 * Copyright (c) 2015-2020 Tomasz Lemiech <szpajder@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstring>
#include <cstdlib>
#include <cassert>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>
#include <syslog.h>
#include "rtl_airband.h"

static char *err;

static inline void mixer_set_error(const char *msg) {
	err = strdup(msg);
}

const char *mixer_get_error() {
	return (const char *)err;
}

mixer_t *getmixerbyname(const char *name) {
	for(int i = 0; i < mixer_count; i++) {
		if(!strcmp(mixers[i].name, name)) {
			debug_print("%s found at %d\n", name, i);
			return &mixers[i];
		}
	}
	debug_print("%s not found\n", name);
	return NULL;
}

void mixer_disable(mixer_t *mixer) {
	mixer->enabled = false;
	disable_channel_outputs(&mixer->channel);
}

int mixer_connect_input(mixer_t *mixer, float ampfactor, float balance) {
	if(!mixer) {
		mixer_set_error("mixer is undefined");
		return(-1);
	}
	int i = mixer->input_count;
	if(i >= MAX_MIXINPUTS) {
		mixer_set_error("too many inputs");
		return(-1);
	}
	mixer->inputs[i].wavein = (float *)XCALLOC(WAVE_LEN, sizeof(float));
	if((pthread_mutex_init(&mixer->inputs[i].mutex, NULL)) != 0) {
		mixer_set_error("failed to initialize input mutex");
		return(-1);
	}
	mixer->inputs[i].ampfactor = ampfactor;
	mixer->inputs[i].ampl = fminf(1.0f, 1.0f - balance);
	mixer->inputs[i].ampr = fminf(1.0f, 1.0f + balance);
	if(balance != 0.0f)
		mixer->channel.mode = MM_STEREO;
	mixer->inputs[i].ready = false;
	SET_BIT(mixer->input_mask, i);
	SET_BIT(mixer->inputs_todo, i);
	mixer->enabled = true;
	debug_print("ampfactor=%.1f ampl=%.1f ampr=%.1f\n", mixer->inputs[i].ampfactor, mixer->inputs[i].ampl, mixer->inputs[i].ampr);
	return(mixer->input_count++);
}

void mixer_disable_input(mixer_t *mixer, int input_idx) {
	assert(mixer);
	assert(input_idx < mixer->input_count);
	RESET_BIT(mixer->input_mask, input_idx);
	if(mixer->input_mask == 0) {
		log(LOG_NOTICE, "Disabling mixer '%s' - all inputs died\n", mixer->name);
		mixer_disable(mixer);
	}
}

void mixer_put_samples(mixer_t *mixer, int input_idx, float *samples, unsigned int len) {
	assert(mixer);
	assert(samples);
	assert(input_idx < mixer->input_count);
	mixinput_t *input = &mixer->inputs[input_idx];
	pthread_mutex_lock(&input->mutex);
	memcpy(input->wavein, samples, len * sizeof(float));
	if(DEBUG && input->ready == true)
		debug_print("input %d overrun\n", input_idx);
	input->ready = true;
	pthread_mutex_unlock(&input->mutex);
}

static bool mix_waveforms(float *sum, float *in, float mult, int size) {
	if(mult == 0.0f) return false;
	bool squelch_open = false;
	for(int s = 0; s < size; s++) {
		sum[s] += in[s] * mult;
	}
	for(int s = 0; s < size; s++) {
		if(in[s] != 0.0f) {
			squelch_open = true;
			break;
		}
	}
	return squelch_open;
}

/* Samples are delivered to mixer inputs in batches of WAVE_BATCH size (default 1000, ie. 1/8 secs
 * of audio). mixer_thread emits mixed audio in batches of the same size, but the loop runs
 * twice more often (MIX_DIVISOR = 2) in order to accomodate for any possible input jitter
 * caused by irregular process scheduling, RTL clock instability, etc. For this purpose
 * we allow each input batch to become delayed by 1/16 secs (max). This is accomplished by
 * the mixer->interval counter, which counts from 2 to 0:
 * - 2 - initial state after mixed audio output. We don't expect inputs to be ready yet,
 *       but we check their readiness anyway.
 * - 1 - here we expect most (if not all) inputs to be ready, so we mix them. If there are no
 *       inputs left to handle in this WAVE_BATCH interval, we emit the mixed audio and reset
 *       mixer->interval to the initial state (2).
 * - 0 - here we expect to get output from all delayed inputs, which were not ready in the
 *       interval. Any input which is still not ready, is skipped (filled with 0s), because
 *       here we must emit the mixed audio to keep the desired audio bitrate.
 */
void *mixer_thread(void *) {
	struct timeval ts, te;
	int interval_usec = 1e+6 * WAVE_BATCH / WAVE_RATE / MIX_DIVISOR;
	if(mixer_count <= 0) return 0;
	if(DEBUG) gettimeofday(&ts, NULL);
	while(!do_exit) {
		usleep(interval_usec);
		if(do_exit) return 0;
		for(int i = 0; i < mixer_count; i++) {
			mixer_t *mixer = mixers + i;
			if(mixer->enabled == false) continue;
			channel_t *channel = &mixer->channel;

			if(channel->state == CH_READY) {		// previous output not yet handled by output thread
				if(--mixer->interval > 0) {
					continue;
				} else {
					debug_print("mixer[%d]: output channel overrun\n", i);
				}
			}

			for(int j = 0; j < mixer->input_count; j++) {
				mixinput_t *input = mixer->inputs + j;
				pthread_mutex_lock(&input->mutex);
				if(IS_SET(mixer->inputs_todo & mixer->input_mask, j) && input->ready) {
					if(channel->state == CH_DIRTY) {
						memset(channel->waveout, 0, WAVE_BATCH * sizeof(float));
						if(channel->mode == MM_STEREO)
							memset(channel->waveout_r, 0, WAVE_BATCH * sizeof(float));
						channel->axcindicate = NO_SIGNAL;
						channel->state = CH_WORKING;
					}
					debug_bulk_print("mixer[%d]: ampleft=%.1f ampright=%.1f\n", i, input->ampfactor * input->ampl, input->ampfactor * input->ampr);
					/* left channel */
					if(mix_waveforms(channel->waveout, input->wavein, input->ampfactor * input->ampl, WAVE_BATCH))
						channel->axcindicate = SIGNAL;
					/* right channel */
					if(channel->mode == MM_STEREO) {
						if(mix_waveforms(channel->waveout_r, input->wavein, input->ampfactor * input->ampr, WAVE_BATCH))
							channel->axcindicate = SIGNAL;
					}
					input->ready = false;
					RESET_BIT(mixer->inputs_todo, j);
				}
				pthread_mutex_unlock(&input->mutex);
			}

			if((mixer->inputs_todo & mixer->input_mask) == 0 || mixer->interval == 0) {	// all good inputs handled or last interval passed
		        if(DEBUG) {
		            gettimeofday(&te, NULL);
		            debug_bulk_print("mixerinput: %lu.%lu %lu int=%d inp_unhandled=0x%02x inp_mask=0x%02x\n",
						te.tv_sec, te.tv_usec, (te.tv_sec - ts.tv_sec) * 1000000UL + te.tv_usec - ts.tv_usec,
						mixer->interval, mixer->inputs_todo, mixer->input_mask);
		            ts.tv_sec = te.tv_sec;
	        	    ts.tv_usec = te.tv_usec;
			    }
				channel->state = CH_READY;
				safe_cond_signal(&mp3_cond, &mp3_mutex);
				mixer->interval = MIX_DIVISOR;
				mixer->inputs_todo = ONES(mixer->input_count);
			} else {
				mixer->interval--;
			}
		}
	}
	return 0;
}

// vim: ts=4
