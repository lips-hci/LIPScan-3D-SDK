#pragma once

#ifdef LIPSCONFIG_EXPORTS
#define LIPSCONFIG_API __declspec(dllexport)
#else
#define LIPSCONFIG_API __declspec(dllimport)
#endif

#include <iostream>
#include <boost\property_tree\json_parser.hpp>
#include <boost\property_tree\xml_parser.hpp>
#include <boost\property_tree\detail\xml_parser_writer_settings.hpp>

using namespace std;
using namespace boost::property_tree;
using namespace boost::property_tree::xml_parser;

namespace lips
{
	class LIPSCONFIG_API XmlParser3D
	{
	public:
		XmlParser3D();
		~XmlParser3D();
		boost::property_tree::ptree xmltree;
		boost::property_tree::ptree Read(std::string filename);
		std::string get(std::string xpath);
		void Write(std::string filename, ptree tree);
	};

	class LIPSCONFIG_API JsonParser
	{
	public:
		JsonParser();
		~JsonParser();
		boost::property_tree::ptree jsontree;
		boost::property_tree::ptree Read(std::string filename);
		void Write(std::string filename, ptree tree);
	};

}