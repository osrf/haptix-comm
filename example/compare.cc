/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include <iostream>
#include <fstream>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

namespace validation
{
  //////////////////////////////////////////////////
  /// \brief Linear interpolation.
  double lerp(const double _t0, const double _v0,
              const double _t1, const double _v1,
              const double _t)
  {
    // Normalize t.
    double t = (_t - _t0) / (_t1 - _t0);
    return (1 - t) * _v0 + t * _v1;
  }

  //////////////////////////////////////////////////
  /// \brief ToDo.
  double compare(const double _t1, const double _v1,
                 const double _t2, const double _v2,
                 const double _prevT2, const double _prevV2)
  {
    double tmpV2 = lerp(_prevT2, _prevV2, _t2, _v2, _t1);
    double error1 = std::pow(_v1 - tmpV2, 2.0);
    std::cout << tmpV2 << std::endl;
    std::cout << error1 << std::endl << std::endl;
    return error1;
  }

  //////////////////////////////////////////////////
  /// \brief ToDo.
  double compareFiles(const std::string &_path1,
                      const std::string &_path2,
                      const double _maxError)
  {
    std::string line1;
    std::string line2;
    std::vector<std::string> prevValues1;
    std::vector<std::string> prevValues2;
    std::vector<std::string> values2;
    std::ifstream f1;
    std::ifstream f2;
    double initialT1;
    double initialT2;
    double t2 = 0.0;
    f1.open(_path1);
    f2.open(_path2);
    int counter = 0;
    double error = 0.0;

    if (!f1.is_open() || !f2.is_open())
      return -1;

    while (std::getline(f1, line1))
    {
      std::vector<std::string> values1;
      double prevT2;
      boost::split(values1, line1, boost::is_any_of(" \t"));
      double t = std::stod(values1.at(0));

      if (prevValues1.empty())
      {
        initialT1 = t;
      }
      else
      {
        double t1 = t - initialT1;

        bool more = true;
        while (t2 < t1 && more)
        {
          more = std::getline(f2, line2);
          if (more)
          {
            prevValues2 = values2;
            boost::split(values2, line2, boost::is_any_of(" \t"));
            // Get the time.
            t = std::stod(values2.at(0));

            if (prevValues2.empty())
            {
              initialT2 = t;
              prevValues2 = values2;
            }
            else
            {
              prevT2 = t2;
              t2 = t - initialT2;
            }
          }
        }

        if (!more)
          return 0;

        for (unsigned int i = 1; i < values1.size(); ++i)
        {
          double v1 = std::stod(values1.at(i));
          double v2 = std::stod(values2.at(i));
          double prevV2 = std::stod(prevValues2.at(i));
          error += compare(t1, v1, t2, v2, prevT2, prevV2);
          ++counter;
        }
      }

      prevValues1 = values1;
    }

    double avgErrorRad = error / counter;
    double avgErrorDeg = avgErrorRad * 180 / 3.14159;
    std::cout << "Average error: " << avgErrorRad << " rads" << std::endl;
    std::cout << "Average error: " << avgErrorDeg << " degs" << std::endl;

    // Normalize the error (degs).
    double normErr = std::max(0.0, std::min(avgErrorDeg / _maxError, 1.0));

    f1.close();
    f2.close();

    return normErr;
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  const double kMaxError = 30.0;
  double normError = validation::compareFiles(
    "validation_1.log", "validation_2.log", kMaxError);
  std::cout << "Normalized error: " << normError << std::endl;
}
