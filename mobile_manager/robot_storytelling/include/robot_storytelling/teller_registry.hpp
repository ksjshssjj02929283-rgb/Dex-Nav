// Copyright 2026 DEXE Robot Engineer
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

#ifndef ROBOT_STORYTELLING__TELLER_REGISTRY_HPP_
#define ROBOT_STORYTELLING__TELLER_REGISTRY_HPP_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "robot_storytelling/teller_base.hpp"

namespace robot_storytelling
{

// Teller 工厂注册表（单例）。对齐 Apollo storytelling 团队自己在
// raw_sources/8. Apollo Storytelling 模块梳理.md §十一 要点6 里给出的建议：
// "Apollo 当前 teller 数量少用硬编码注册够用；机器人室内场景 story 类型
// 预计 20+，一开始就引入 Factory 模式，避免走 Apollo 走过的
// 小数量→硬编码→增长后重构 的路径。"
//
// 用法：每个 teller 的 .cpp 末尾写一行 ROBOT_STORYTELLER_REGISTER(ClassName)，
// 该 teller 即可通过字符串名字（= 类名）在 enabled_tellers 参数里启用，
// 不需要在 StorytellingNode 里手写 if/else 或 new 语句。
class TellerRegistry
{
public:
  using Creator = std::function<std::unique_ptr<TellerBase>()>;

  static TellerRegistry & Instance();

  // 供 ROBOT_STORYTELLER_REGISTER 宏在静态初始化阶段调用。
  void RegisterCreator(const std::string & name, Creator creator);

  // 按名字创建一个 teller 实例；名字未注册时返回 nullptr（调用方负责报错/告警）。
  std::unique_ptr<TellerBase> Create(const std::string & name) const;

  // 当前已注册（编译进本可执行文件）的全部 teller 名字，用于日志/自省。
  std::vector<std::string> RegisteredNames() const;

private:
  TellerRegistry() = default;
  std::unordered_map<std::string, Creator> creators_;
};

}  // namespace robot_storytelling

// 自注册宏：在 teller 的 .cpp 文件末尾调用一次即可，无需修改 StorytellingNode。
// 利用 C++ 静态对象构造在 main() 之前执行的特性完成注册（与常见 pluginlib /
// gflags 风格的自注册宏思路一致）。
#define ROBOT_STORYTELLER_REGISTER(ClassName) \
  namespace \
  { \
  struct ClassName ## Registrar \
  { \
    ClassName ## Registrar() \
    { \
      ::robot_storytelling::TellerRegistry::Instance().RegisterCreator( \
        #ClassName, \
        []() -> std::unique_ptr<::robot_storytelling::TellerBase> { \
          return std::make_unique<::robot_storytelling::ClassName>(); \
        }); \
    } \
  }; \
  static ClassName ## Registrar g_ ## ClassName ## _registrar_instance; \
  }  // namespace

#endif  // ROBOT_STORYTELLING__TELLER_REGISTRY_HPP_
