#include <itomp_exec/nlp/language_model.h>


namespace itomp_exec
{

Token::Token()
{
}

void Token::setBasicDependencyParent(Token* parent, const std::string& type)
{
    basic_dependency_parent_ = parent;
    basic_dependency_parent_type_ = type;
}

void Token::addBasicDependencyChild(Token* child)
{
    basic_dependency_children_.push_back(child);
}


Sentence::Sentence()
{
    tokens_[0] = new Token(); // root
}

Sentence::~Sentence()
{
    for (std::map<int, Token*>::iterator it = tokens_.begin(); it != tokens_.end(); it++)
        delete it->second;
}

void Sentence::addBasicDependency(int parent_id, int child_id, const std::string& type)
{
    tokens_[parent_id]->addBasicDependencyChild( tokens_[child_id] );
    tokens_[child_id]->setBasicDependencyParent( tokens_[parent_id], type );
}


LanguageModel::LanguageModel()
{
}

LanguageModel::~LanguageModel()
{
    for (std::map<int, Sentence*>::iterator it = sentences_.begin(); it != sentences_.end(); it++)
        delete it->second;
}

}
