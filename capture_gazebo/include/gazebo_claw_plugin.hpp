/*
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
 * Author: Luís Abrantes
 */

#ifndef GAZEBO_CLAW_PLUGIN_HPP
#define GAZEBO_CLAW_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo
{
  class GazeboClawPluginPrivate;

  class GazeboClawPlugin : public gazebo::ModelPlugin
  {
    public:
      GazeboClawPlugin();

      virtual ~GazeboClawPlugin();

      // Loads the plugin
      void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

    private:
      std::unique_ptr<GazeboClawPluginPrivate> impl_;
  };
}
#endif // GAZEBO_CLAW_PLUGIN_HPP
