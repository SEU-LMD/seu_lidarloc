#pragma once

// C++ includes
#include <iostream>
#include <string>

/// The namespace for the example code demonstrating doxystrap.
namespace doxystrap {

/// Provide an example on how this class is documented with `doxystrap`.
class Example
{
public:
    /// Construct a default Example instance.
    Example();

    /// Construct a custom Example instance.
    /// @param name The name of the example.
    /// @param id The id number of the example.
    Example(std::string name, int id);

    /// Destroy this instance.
    virtual ~Example();

    /// Set the name of the example.
    /// @param name The name of the example.
    /// @param id The id number of the example.
    void setName(std::string name, int id);

    /// Return the name of the example.
    std::string name() const;

    /// Return the id of the example.
    int id() const;

    /// Print a value of any type.
    template<typename T>
    void print(T val) {
        std::cout << val << std::endl;
    }

private:
    /// The name of the example.
    std::string m_name;

    /// The id of the example.
    int m_id;
};

} // namespace doxystrap
