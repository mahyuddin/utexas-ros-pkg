/**
 * \file  PatternParser.h
 * \brief  Reads in patterns file from YAML Syntax
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

  struct Pattern {
    std::string name;
    std::string patternFile;
    Vec3f location;
    float size;
  };

  void operator >> (const YAML::Node& node, Vec3f& v) {
     node[0] >> v.x;
     node[1] >> v.y;
     node[2] >> v.z;
  }

  void operator >> (const YAML::Node& node, Pattern& p) {
    node["name"] >> p.name;
    node["location"] >> p.location;
    node["patternFile"] >> p.patternFile;
    node["size"] >> p.size;
  }

  void operator >> (const YAML::Node& doc, std::vector<Pattern>& patterns) {
    patterns.clear();
    for (uint16_t i = 0; i < doc.size(); i++) {
      Pattern p;
      doc[i] >> p;
      patterns.push_back(p);
    }
  }

  void readPatternFile(std::string fileName, std::vector<Pattern>& patterns) {
    std::ifstream fin(fileName.c_str());
    YAML::Parser parser(fin);
    YAML::Node doc;
    parser.GetNextDocument(doc);
    doc >> patterns;
    fin.close();
  }

  void writeARPatternFile(std::string fileName, const std::vector<Pattern>& patterns) {

    std::ofstream fout(fileName.c_str());
    fout << "#Autogenerated from YAML file. DO NOT EDIT!!" << std::endl;
    
    // Number of patterns
    fout << "#the number of patterns to be recognized" << std::endl;
    fout << patterns.size() << std::endl;

    uint16_t count = 1;
    BOOST_FOREACH(const Pattern& pattern, patterns) {
      fout << std::endl;
      fout << "#pattern " << count << std::endl;
      fout << pattern.name << std::endl;
      fout << pattern.patternFile << std::endl;
      fout << pattern.size << std::endl;
      fout << "0.0 0.0" << std::endl;
      count++;
    }

    fout.close();
  }

}

#endif /* end of include guard: PATTERNPARSER_KQJF84V6 */
