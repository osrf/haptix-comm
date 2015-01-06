/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef _HAPTIX_GENERATOR_H_
#define _HAPTIX_GENERATOR_H_

#include <google/protobuf/compiler/code_generator.h>
#include <string>

namespace google {
namespace protobuf {
namespace compiler {
namespace cpp {
class GeneratorContext;

/// \brief Google protobuf message generator for haptix::msgs
class HaptixGenerator : public CodeGenerator
{
  public: HaptixGenerator(const std::string &_name);

  public: virtual ~HaptixGenerator();

  public: virtual bool Generate(const FileDescriptor* _file,
                const string& _parameter,
                OutputDirectory *_generator_context,
                string* _error) const;

  // private: GOOGLE_DISALLOW_EVIL_CONSTRUCTORS(HaptixGenerator);
};
} // namespace cpp
} // namespace compiler
} // namespace protobuf
} // namespace google
#endif
