// Copyright 2023 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*****************************************************************************************************************
 *
 * 	 Authors: Jaycee Lock
 *
 ******************************************************************************************************************/

#pragma once

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/ssl.hpp>
#include <boost/beast/version.hpp>
#include <http_client/http_session.hpp>
#include <http_client/ssl_http_session.hpp>
#include <smacc2/smacc.hpp>
#include <smacc2/smacc_client.hpp>
#include <string>
#include <thread>

namespace cl_http
{
class ClHttp : public smacc2::ISmaccClient
{
  class Server
  {
  public:
    explicit Server(const std::string & server_name) : server_name_{server_name}, ssl_{true}
    {
      if (!server_name_.substr(0, 7).compare("http://"))
      {
        ssl_ = false;
        server_name_.erase(0, 7);
      }
      else if (!server_name_.substr(0, 8).compare("https://"))
      {
        server_name_.erase(0, 8);
        ssl_ = true;
      }
    }

    bool isSSL() const { return ssl_; }

    std::string getPort() const { return ssl_ ? "443" : "80"; }

    std::string getServerName() const { return server_name_; }

  private:
    std::string server_name_;
    bool ssl_;
  };

public:
  enum class kHttpRequestMethod
  {
    GET = static_cast<int>(boost::beast::http::verb::get),
    POST = static_cast<int>(boost::beast::http::verb::post),
    PUT = static_cast<int>(boost::beast::http::verb::put),
  };

  using TResponse = http_session_base::TResponse;

  template <typename T>
  boost::signals2::connection onResponseReceived(void (T::*callback)(const TResponse &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onResponseReceived_, callback, object);
  }

  explicit ClHttp(const std::string & server, const int & timeout = 1500);

  virtual ~ClHttp();

  void onInitialize() override;

  void makeRequest(
    const kHttpRequestMethod http_method, const std::string & path = "/",
    const std::string & body = "",
    const std::unordered_map<std::string, std::string> & headers = {});

private:
  const int HTTP_VERSION = 11;

  bool initialized_;
  bool is_ssl_;
  int timeout_;

  Server server_;

  boost::asio::io_context io_context_;
  boost::asio::executor_work_guard<decltype(io_context_)::executor_type> worker_guard_;
  std::thread tcp_connection_runner_;

  boost::asio::ssl::context ssl_context_;

  smacc2::SmaccSignal<void(const TResponse &)> onResponseReceived_;

  std::function<void(TResponse)> callbackHandler = [&](const TResponse & res)
  { onResponseReceived_(res); };
};
}  // namespace cl_http
