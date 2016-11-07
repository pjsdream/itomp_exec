#ifndef ITOMP_EXEC_LANGUAGE_MODEL_H
#define ITOMP_EXEC_LANGUAGE_MODEL_H


#include <string>
#include <map>
#include <vector>


namespace itomp_exec
{

class Token
{
public:

    Token();

    void setBasicDependencyParent(Token* parent, const std::string& type);
    void addBasicDependencyChild(Token* child);

    std::string word;
    std::string lemma;
    std::string pos;

private:

    std::string basic_dependency_parent_type_;

    Token* basic_dependency_parent_;
    std::vector<Token*> basic_dependency_children_;
};


class Sentence
{
public:

    Sentence();
    ~Sentence();

    inline Token* addToken(int id)
    {
        Token* token = new Token();
        tokens_[id] = token;
        return token;
    }

    void addBasicDependency(int parent_id, int child_id, const std::string& type);

private:

    std::map<int, Token*> tokens_;
};


class LanguageModel
{
public:

    LanguageModel();
    ~LanguageModel();

    inline Sentence* addSentence(int id)
    {
        Sentence* sentence = new Sentence();
        sentences_[id] = sentence;
        return sentence;
    }

private:

    std::map<int, Sentence*> sentences_;
};

}


#endif // ITOMP_EXEC_LANGUAGE_MODEL_H
