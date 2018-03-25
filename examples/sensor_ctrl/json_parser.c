/**
 * @file   sensor_ctrl.c
 * @author Jean-Nicolas Graux
 *
 * @brief  my little sensor controller.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "jsmn.h"

//#define DEBUG
#include "trace.h"

#define NB_TOKENS 64
jsmntok_t t[NB_TOKENS];

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
	if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
			strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
		return 0;
	}
	return -1;
}

int json_get_value_from_key(char *http_resp, char* key, char *value, int max_len)
{
	int i, r, len, ret = -1;
	jsmn_parser p;
	char *json_resp = http_resp;

	len = strlen(http_resp);
	for (i = 0; i < len ; i++) {
		if (http_resp[i] == '{') {
			json_resp = http_resp + i;
			break;
		}
	}

	if (*json_resp != '{') {
		INFO("failed to find a bracket\n");
		goto end;
	}

	jsmn_init(&p);
	r = jsmn_parse(&p, json_resp, strlen(json_resp), t, NB_TOKENS);
	if (r < 0) {
		INFO("Failed to parse JSON: %d\n", r);
		goto end;
	}

	/* Assume the top-level element is an object */
	if (r < 1 || t[0].type != JSMN_OBJECT) {
		INFO("Object expected\n");
		goto end;
	}

	for (i = 1; i < r; i++)
		if (jsoneq(json_resp, &t[i], key) == 0) {
			len = t[i + 1].end - t[i + 1].start;
			if (len > max_len - 1) {
				INFO("len is too big: %d\n", len);
				goto end;
			}
			memcpy(value, json_resp + t[i + 1].start, len);
			value[len] = '\0';
			ret = 0;
			goto end;
		}
end:
	return ret;
}


