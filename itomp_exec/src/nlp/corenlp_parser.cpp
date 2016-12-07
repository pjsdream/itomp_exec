#include <itomp_exec/nlp/corenlp_parser.h>


namespace itomp_exec
{

CoreNLPParser::CoreNLPParser(const std::string& xml_string)
{
    language_model_ = new LanguageModel();

    tinyxml2::XMLDocument xml_parser;
    tinyxml2::XMLError error = xml_parser.Parse(xml_string.c_str());

    if (error == 0)
    {
        tinyxml2::XMLElement* document_node = xml_parser.RootElement()->FirstChildElement();
        tinyxml2::XMLElement* sentences_node = document_node->FirstChildElement("sentences");

        parseSentences(sentences_node);

        if (document_node->FirstChildElement("coreference") != 0)
        {
            tinyxml2::XMLElement* coreference_node = document_node->FirstChildElement("coreference")->FirstChildElement("coreference");
            parseCoreference(coreference_node);
        }
    }

    else
    {
        fprintf(stderr, "Error occurred while parsing xml string\n");
    }
}

LanguageModel* CoreNLPParser::getLanguageModel()
{
    return language_model_;
}

void CoreNLPParser::parseSentence(tinyxml2::XMLElement* node, Sentence* sentence)
{
    // tokens
    tinyxml2::XMLElement* tokens_node = node->FirstChildElement("tokens");
    for (tinyxml2::XMLElement* token_node = tokens_node->FirstChildElement("token"); token_node != 0; token_node = token_node->NextSiblingElement("token"))
    {
        const int token_id = atoi( token_node->Attribute("id") );
        Token* token = sentence->addToken(token_id);

        token->word = token_node->FirstChildElement("word")->GetText();
        token->lemma = token_node->FirstChildElement("lemma")->GetText();
        token->pos = token_node->FirstChildElement("POS")->GetText();
    }

    // dependencies
    for (tinyxml2::XMLElement* basic_dependencies_node = node->FirstChildElement("dependencies"); basic_dependencies_node != 0; basic_dependencies_node = basic_dependencies_node->NextSiblingElement("dependencies"))
    {
        if (basic_dependencies_node->Attribute("type") == std::string("basic-dependencies"))
        {
            for (tinyxml2::XMLElement* dep_node = basic_dependencies_node->FirstChildElement("dep"); dep_node != 0; dep_node = dep_node->NextSiblingElement("dep"))
            {
                const int parent_id = atoi( dep_node->FirstChildElement("governor")->Attribute("idx") );
                const int child_id = atoi( dep_node->FirstChildElement("dependent")->Attribute("idx") );
                std::string type = dep_node->Attribute("type");

                sentence->addBasicDependency(parent_id, child_id, type);
            }
        }
    }
}

void CoreNLPParser::parseSentences(tinyxml2::XMLElement* node)
{
    for (tinyxml2::XMLElement* sentence_node = node->FirstChildElement("sentence"); sentence_node != 0; sentence_node = sentence_node->NextSiblingElement("sentence"))
    {
        const int sentence_id = atoi( sentence_node->Attribute("id") );
        Sentence* sentence = language_model_->addSentence(sentence_id);
        parseSentence(sentence_node, sentence);
    }
}

void CoreNLPParser::parseCoreference(tinyxml2::XMLElement* node)
{
}

}
