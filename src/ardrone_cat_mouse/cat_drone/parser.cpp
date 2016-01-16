#include "parser.h"

namespace cat {

Parser::Parser(std::string tag) {
    tag_ = tag;
}

Parser::~Parser()
{
    //dtor
}

void Parser::AddTextValue(const xmlpp::Node* n, std::vector<std::string>& sv) {
    xmlpp::Node *newnode = *(n->get_children().begin());
	const xmlpp::TextNode* nodeTextValue = dynamic_cast
          <const xmlpp::TextNode*>(*((n)->get_children().begin())); //Error de ejecucion
    std::string s_temp(nodeTextValue->get_content());
    sv.push_back(s_temp);

}

std::vector<std::string> Parser::ReadInfo(const xmlpp::Node* node) {
	xmlpp::Node::NodeList list = node->get_children();
    std::vector<std::string> returned;
	if (list.size() > 1) { // Not a TextNode
        returned.reserve(list.size());
		for(xmlpp::Node::NodeList::iterator iter = list.begin();
                iter != list.end(); ++iter) {
            if ((*iter)->get_name().compare("text") != 0)
                AddTextValue(*iter, returned);
        }
    } else { // TextNode
        AddTextValue(node, returned);
    }
    return returned;
}

void Parser::Parse(const xmlpp::Node* node, CatConfig *conf) {
    const xmlpp::TextNode* nodeText =
            dynamic_cast<const xmlpp::TextNode*>(node);
    const xmlpp::CommentNode* nodeComment =
            dynamic_cast<const xmlpp::CommentNode*>(node);
    if((nodeText && nodeText->is_white_space()))
        return;
    Glib::ustring nodename = node->get_name();

  	if(!nodeText && !nodeComment && !nodename.empty()) {
	    const Glib::ustring namespace_prefix = node->get_namespace_prefix();
		if(namespace_prefix.empty()){

            if(nodename.compare(tag_)==0) {
                std::vector<std::string> vTemp = ReadInfo(node);
                conf->AddInfo(vTemp);
            } else {
    			xmlpp::Node::NodeList list = node->get_children();
			    if (list.size() > 1) {
				    for(xmlpp::Node::NodeList::iterator iter = list.begin();
                            iter != list.end(); ++iter)
                        Parse(*iter, conf);
			    }
			}
		}
  	}
}

int Parser::ReadFile(std::string filepath, CatConfig *conf) {
	#ifdef LIBXMLCPP_EXCEPTIONS_ENABLED
	try
	{
  	#endif //LIBXMLCPP_EXCEPTIONS_ENABLED
		xmlpp::DomParser parser;
    	parser.set_substitute_entities();
    	parser.parse_file(filepath);
    	if(parser)
    	{
      		const xmlpp::Node* pNode = parser.get_document()->get_root_node();
	    Parse(pNode, conf);
   	 	}
		#ifdef LIBXMLCPP_EXCEPTIONS_ENABLED
	  	}
  	catch(const std::exception& ex)
  	{
	    std::cout << "Exception caught: " << ex.what() << std::endl;
  	}
  	#endif //LIBXMLCPP_EXCEPTIONS_ENABLED
  	return 0;
}

} //cat
