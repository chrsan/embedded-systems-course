#include "http.h"

#include "esp_err.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep.
#include "freertos/queue.h"
#include "json.h"
#include "measurement.h"
#include "message.h"

static esp_err_t http_server_handler(httpd_req_t* req) {
  httpd_req_t* req_copy = NULL;
  esp_err_t ret = httpd_req_async_handler_begin(req, &req_copy);
  if (ret != ESP_OK) {
    return ret;
  }

  struct MainQueueMessage message = {
      .type = MAIN_QUEUE_MESSAGE_TYPE_HTTP_REQUEST,
      .data =
          {
              .http_req = req_copy,
          },
  };

  QueueHandle_t main_queue = req->user_ctx;
  if (xQueueSendToBack(main_queue, &message, pdMS_TO_TICKS(1000)) == pdFALSE) {
    httpd_resp_set_status(req, "503 Busy");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "Looks like we're busy...");
    httpd_req_async_handler_complete(req_copy);
    return ESP_FAIL;
  }

  return ESP_OK;
}

httpd_handle_t http_server = NULL;

esp_err_t http_server_setup(QueueHandle_t main_queue) {
  httpd_config_t conf = HTTPD_DEFAULT_CONFIG();
  conf.max_open_sockets = 2;
  conf.lru_purge_enable = true;

  esp_err_t ret = httpd_start(&http_server, &conf);
  if (ret != ESP_OK) {
    return ret;
  }

  const httpd_uri_t uri = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = http_server_handler,
      .user_ctx = main_queue,
  };

  return httpd_register_uri_handler(http_server, &uri);
}

struct JsonState {
  httpd_req_t* http_req;
  esp_err_t err;
};

static void json_flush(char* buf, void* priv) {
  struct JsonState* json_state = priv;
  if (json_state->err == ESP_OK) {
    json_state->err = httpd_resp_sendstr_chunk(json_state->http_req, buf);
  }
}

esp_err_t http_handle_req(httpd_req_t* req,
                          const struct MeasurementState* state) {
  esp_err_t ret;
  if ((ret = httpd_resp_set_type(req, "application/json")) != ESP_OK) {
    return ret;
  }

  struct JsonState json_state = {
      .http_req = req,
      .err = ESP_OK,
  };

  char buf[32];
  json_gen_str_t jstr;
  json_gen_str_start(&jstr, buf, sizeof(buf), json_flush, &json_state);
  json_gen(&jstr, state);
  json_gen_str_end(&jstr);

  if (json_state.err != ESP_OK) {
    return json_state.err;
  }

  if ((ret = httpd_resp_sendstr_chunk(req, NULL)) != ESP_OK) {
    return ret;
  }

  return httpd_req_async_handler_complete(req);
}
