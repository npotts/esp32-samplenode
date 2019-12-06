#include <stdio.h>

#include "shared-data.h"

const char * TAG = "wx-data";

sample_t update_value_f(float v, esp_err_t error) {
	sample_t samp = { .error = error, .type = FloatSampleType, .whence = esp_log_timestamp(), .sample.f = v};
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