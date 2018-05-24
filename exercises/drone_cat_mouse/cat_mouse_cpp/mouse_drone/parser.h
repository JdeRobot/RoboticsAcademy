#ifndef PATHPARSER_H
#define PATHPARSER_H

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <libxml++/libxml++.h>
//#include "utils.h"
#include "config/drone_config.h"
#include <fstream>
#include <iostream>
#include <string>

namespace mouse {

class Parser {
public:
	Parser(std::string);
    int ReadFile(std::string filepath, DroneConfig *conf);
    virtual ~Parser();
private:
	void Parse(const xmlpp::Node* node, DroneConfig *conf);
    std::vector<std::string> ReadInfo(const xmlpp::Node* node);
    void AddTextValue(const xmlpp::Node*, std::vector<std::string>&);
    std::string tag_;
};

}

#endif // PATHPARSER_H
