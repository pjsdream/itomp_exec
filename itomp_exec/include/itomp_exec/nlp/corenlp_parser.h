#ifndef ITOMP_EXEC_CORENLP_PARSER_H
#define ITOMP_EXEC_CORENLP_PARSER_H


#include <itomp_exec/nlp/language_model.h>

#include <string>

#include <tinyxml2/tinyxml2.h>

namespace itomp_exec
{

class CoreNLPParser
{
public:

    CoreNLPParser(const std::string& xml_string);

    LanguageModel* getLanguageModel();

private:

    LanguageModel* language_model_;

    void parseSentence(tinyxml2::XMLElement* node, Sentence* sentence);
    void parseSentences(tinyxml2::XMLElement* node);
    void parseCoreference(tinyxml2::XMLElement* node);
};

}


#endif // ITOMP_EXEC_CORENLP_PARSER_H
