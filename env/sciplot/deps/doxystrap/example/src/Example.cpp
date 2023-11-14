#include "Example.hpp"

namespace doxystrap {

Example::Example()
{}

Example::Example(std::string name, int id)
: m_name(name), m_id(id)
{}

Example::~Example()
{}

void Example::setName(std::string name, int id)
{
    m_name = name;
    m_id = id;
}

std::string Example::name() const
{
    return m_name;
}

int Example::id() const
{
    return m_id;
}

} // namespace doxystrap
