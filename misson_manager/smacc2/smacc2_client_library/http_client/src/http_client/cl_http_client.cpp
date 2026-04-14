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

#include <http_client/cl_http_client.hpp>

namespace cl_http
{
ClHttp::ClHttp(const std::string & server_name, const int & timeout)
: initialized_{false},
  timeout_{timeout},
  server_{server_name},
  worker_guard_{boost::asio::make_work_guard(io_context_.get_executor())},
  ssl_context_{boost::asio::ssl::context::tlsv13_client}
{
  ssl_context_.set_default_verify_paths();
  ssl_context_.set_verify_mode(boost::asio::ssl::verify_peer);
}

ClHttp::~ClHttp()
{
  worker_guard_.reset();
  tcp_connection_runner_.join();
}

void ClHttp::onInitialize()
{
  if (!initialized_)
  {
    tcp_connection_runner_ = std::thread{[&]() { io_context_.run(); }};
    this->initialized_ = true;
  }
}

void ClHttp::makeRequest(
  const kHttpRequestMethod http_method, const std::string & path, const std::string & body,
  const std::unordered_map<std::string, std::string> & headers)
{
  auto path_used = path;
  if (path[0] != '/')
  {
    std::reverse(path_used.begin(), path_used.end());
    path_used += '/';
    std::reverse(path_used.begin(), path_used.end());
  }

  RCLCPP_INFO(this->getLogger(), "SSL? %d", server_.isSSL());
  RCLCPP_INFO(this->getLogger(), "Server %s", server_.getServerName().c_str());
  RCLCPP_INFO(this->getLogger(), "Path %s", path_used.c_str());
  RCLCPP_INFO(this->getLogger(), "Port %s", server_.getPort().c_str());

  std::shared_ptr<http_session_base> http_session_ptr;

  if (server_.isSSL())
  {
    http_session_ptr = std::make_shared<ssl_http_session>(
      boost::asio::make_strand(io_context_), ssl_context_, callbackHandler);
  }
  else
  {
    http_session_ptr =
      std::make_shared<http_session>(boost::asio::make_strand(io_context_), callbackHandler);
  }

  http_session_ptr->setBody(body);
  http_session_ptr->setHeaders(headers);
  http_session_ptr->run(
    server_.getServerName(), path_used, static_cast<boost::beast::http::verb>(http_method),
    HTTP_VERSION);
}
}  // namespace cl_http
