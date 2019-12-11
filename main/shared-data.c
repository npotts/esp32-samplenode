/* Dual use Weather and Splot Phase Energy Meter

Intended to be used with CircuitSetup.us Split Phase Energy Meter and some extra tacked on sensors.

The MIT License (MIT)
Copyright (c) 2019 npotts
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.  
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>

#include "shared-data.h"
#include "mqtt.client.h"

const char * TAG = "wx-data";

sample_t update_value_f(double v, esp_err_t error) {
	sample_t samp = { .error = error, .type = DoubleSampleType, .whence = esp_log_timestamp(), .sample.f = v};
	return samp;
}
sample_t update_value_i(int v, esp_err_t error) {
	sample_t samp = { .error = error, .type = IntSampleType, .whence = esp_log_timestamp(), .sample.f = v};
	return samp;
}

int json_sample_t(char * buffer, int n, sample_t t) {
	/*Return sample_t as json obect*/
	int w = snprintf(buffer, n, "{\"t\":%d,\"v\":", t.whence);
	if (t.error != ESP_OK)
		return w + snprintf(buffer+w, n-w, "null}");
	if (t.type == IntSampleType)
		return w + snprintf(buffer+w, n-w, "%d}", t.sample.i);
	return w + snprintf(buffer+w, n-w, "%f}", t.sample.f);
}

int json_wx_data_t(char * buffer, int n, wx_data_t wx) {
	/*Attempt to render out w as a nested json object */
	int w = snprintf(buffer, n, "{\"p\":");
	w += json_sample_t(buffer+w, n-w, wx.p);
	w += snprintf(buffer+w, n+w, ",\"pt\":");
	w += json_sample_t(buffer+w, n-w, wx.pt);
	w += snprintf(buffer+w, n+w, ",\"rh\":");
	w += json_sample_t(buffer+w, n-w, wx.rh);
	w += snprintf(buffer+w, n+w, ",\"rht\":");
	w += json_sample_t(buffer+w, n-w, wx.rht);
	w += snprintf(buffer+w, n+w, "}");
	return w;
}

void broadcast_sample(sample_t s, char *topic) {
    char b[256];
    int n = json_sample_t(b, sizeof(b), s);
    if (mqtt_publish_msg(topic, b, n) == -1) {
        ESP_LOGE(TAG, "Unable to publish heartbeat");
    }
}