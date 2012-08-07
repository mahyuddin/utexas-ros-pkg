/**
 * \file  point.h
 * \brief  Reads in points file from YAML Syntax
 *
 * Some utility functions while dealing with AR Patterns
 * 
 * \author  Piyush Khandelwal (piyushk), piyushk@cs.utexas.edu
 * Copyright (C) 2011, The University of Texas at Austin, Piyush Khandelwal
 *
 * License: Modified BSD License
 *
 * $ Id: 10/02/2011 12:49:43 PM piyushk $
 */

#ifndef PATTERNPARSER_KQJF84V6
#define PATTERNPARSER_KQJF84V6

#include <fstream>
#include <stdint.h>
#include <yaml-cpp/yaml.h>
#include <boost/foreach.hpp>

namespace camera_pose {

  struct Vec3f {
    float x;
    float y;
    float z;
  };

  void operator >> (const YAML::Node& node, Vec3f& v) {
     node[0] >> v.x;
     node[1] >> v.y;
     node[2] >> v.z;
  }

  void operator >> (const YAML::Node& doc, std::vector<Vec3f>& points) {
    points.clear();
    for (uint16_t i = 0; i < doc.size(); i++) {
      Vec3f p;
      doc[i] >> p;
      points.push_back(p);
    }
  }

  void readPointFile(std::string fileName, std::vector<Vec3f>& points) {
    std::ifstream fin(fileName.c_str());
    YAML::Parser parser(fin);
    YAML::Node doc;
    parser.GetNextDocument(doc);
    doc >> points;
    fin.close();
  }

}

#endif /* end of include guard: PATTERNPARSER_KQJF84V6 */
