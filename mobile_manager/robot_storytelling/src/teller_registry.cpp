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

#include "robot_storytelling/teller_registry.hpp"

namespace robot_storytelling
{

TellerRegistry & TellerRegistry::Instance()
{
  static TellerRegistry instance;
  return instance;
}

void TellerRegistry::RegisterCreator(const std::string & name, Creator creator)
{
  creators_[name] = std::move(creator);
}

std::unique_ptr<TellerBase> TellerRegistry::Create(const std::string & name) const
{
  const auto it = creators_.find(name);
  if (it == creators_.end()) {
    return nullptr;
  }
  return it->second();
}

std::vector<std::string> TellerRegistry::RegisteredNames() const
{
  std::vector<std::string> names;
  names.reserve(creators_.size());
  for (const auto & kv : creators_) {
    names.push_back(kv.first);
  }
  return names;
}

}  // namespace robot_storytelling
