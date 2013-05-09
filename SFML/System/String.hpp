////////////////////////////////////////////////////////////
//
// SFML - Simple and Fast Multimedia Library
// Copyright (C) 2007-2012 Laurent Gomila (laurent.gom@gmail.com)
//
// This software is provided 'as-is', without any express or implied warranty.
// In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it freely,
// subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented;
//    you must not claim that you wrote the original software.
//    If you use this software in a product, an acknowledgment
//    in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such,
//    and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
//
////////////////////////////////////////////////////////////

/* !!! THIS IS AN EXTREMELY ALTERED AND PURPOSE-BUILT VERSION OF SFML !!!
 * This distribution is designed to possess only a limited subset of the
 * original library's functionality and to only build on VxWorks 6.3.
 * The original distribution of this software has many more features and
 * supports more platforms.
 */

#ifndef SFML_STRING_HPP
#define SFML_STRING_HPP

#include <locale>
#include <string>


namespace sf {
////////////////////////////////////////////////////////////
/// \brief Utility string class that automatically handles
///        conversions between types and encodings
///
////////////////////////////////////////////////////////////
class String {
public:
    typedef std::basic_string<uint32_t>::iterator       Iterator;      // Iterator type
    typedef std::basic_string<uint32_t>::const_iterator ConstIterator; // Constant iterator type

    static const std::size_t InvalidPos; // Represents an invalid position in the string

    String();

    /* Construct from a single ANSI character and a locale
     *
     * ansiChar is converted to UTF-32 according to the given locale
     */
    String( char ansiChar , const std::locale& locale = std::locale() );

    // Construct from single UTF-32 character
    String( uint32_t utf32Char );

    /* Construct from a null-terminated C-style ANSI string and a locale
     *
     * The source string is converted to UTF-32 according to the given locale.
     */
    String( const char* ansiString , const std::locale& locale = std::locale() );

    /* Construct from an ANSI string and a locale
     *
     * The source string is converted to UTF-32 according to the given locale.
     */
    String( const std::string& ansiString , const std::locale& locale = std::locale() );

    // Construct from a null-terminated C-style UTF-32 string
    String( const uint32_t* utf32String );

    // Construct from an UTF-32 string
    String( const std::basic_string<uint32_t>& utf32String );

    String( const String& copy );

    /* Implicit cast operator to std::string (ANSI string)
     *
     * The current global locale is used for conversion. If you want to
     * explicitly specify a locale, see toAnsiString. Characters that do not
     * fit in the target encoding are discarded from the returned string. This
     * operator is defined for convenience, and is equivalent to calling
     * toAnsiString().
     */
    operator std::string() const;

    std::string toAnsiString(const std::locale& locale = std::locale()) const;

    String& operator =(const String& right);

    String& operator +=(const String& right);

    /* Overload of [] operator to access a character by its position
     *
     * This function provides read-only access to characters.
     * Note: this function doesn't throw if index is out of range.
     */
    uint32_t operator [](std::size_t index) const;

    /* Overload of [] operator to access a character by its position
     *
     * This function provides read and write access to characters.
     * Note: this function doesn't throw if index is out of range.
     */
    uint32_t& operator [](std::size_t index);

    void clear();

    std::size_t getSize() const;

    // Returns 'true' if the string is empty (i.e. contains no character)
    bool isEmpty() const;

    // Removes a sequence of characters starting from position
    void erase( std::size_t position , std::size_t count = 1 );

    // Insert one or more characters into the string starting from position
    void insert( std::size_t position , const String& str );

    ////////////////////////////////////////////////////////////
    /// \brief Find a sequence of one or more characters in the string
    ///
    /// This function searches for the characters of \a str
    /// into the string, starting from \a start.
    ///
    /// \param str   Characters to find
    /// \param start Where to begin searching
    ///
    /// \return Position of \a str in the string, or String::InvalidPos if not found
    ///
    ////////////////////////////////////////////////////////////
    std::size_t find(const String& str, std::size_t start = 0) const;

    const uint32_t* getData() const;

    Iterator begin();

    ConstIterator begin() const;

    Iterator end();

    ConstIterator end() const;

private :

    friend bool operator ==(const String& left, const String& right);
    friend bool operator <(const String& left, const String& right);

    std::basic_string<uint32_t> m_string;
};

bool operator ==(const String& left, const String& right);

bool operator !=(const String& left, const String& right);

bool operator <(const String& left, const String& right);

bool operator >(const String& left, const String& right);

bool operator <=(const String& left, const String& right);

bool operator >=(const String& left, const String& right);

String operator +(const String& left, const String& right);

} // namespace sf


#endif // SFML_STRING_HPP
