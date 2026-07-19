/*
 * Print.h
 *
 * Minimal Arduino Print/String shim - Adafruit_GFX derives from Print (for its
 * write(uint8_t) text-cursor API) and Adafruit_GFX_Button::drawButton() calls
 * print() on it; getTextBounds() has an overload taking a String and another
 * taking a __FlashStringHelper*. Only what those call sites need is implemented.
 */
#ifndef ROBOTDRIVERS_OLEDDRIVERS_ARDUINOCOMPAT_PRINT_H_
#define ROBOTDRIVERS_OLEDDRIVERS_ARDUINOCOMPAT_PRINT_H_

#include <cstddef>
#include <cstdint>
#include <string>

// Real Arduino String is a mutable, ref-counted C-string wrapper; this project only
// ever needs it to exist as a type (Adafruit_GFX::getTextBounds(const String&) must
// compile, nothing in these libraries actually constructs one), so a thin std::string
// wrapper covering .length()/.c_str() is enough.
class String
{
public:
	String() = default;
	String(const char *s) : _s(s ? s : "") {}
	String(char c) : _s(1, c) {}
	size_t length() const { return _s.size(); }
	const char *c_str() const { return _s.c_str(); }

private:
	std::string _s;
};

// Opaque marker type for F("...") flash-string literals. STM32 has no separate
// flash/RAM address space distinction the way AVR does, so F() is just a relabeled
// pointer - no real "read from flash" logic is needed on this platform.
class __FlashStringHelper;
#define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(string_literal))

class Print
{
public:
	virtual ~Print() = default;

	virtual size_t write(uint8_t c) = 0;
	virtual size_t write(const uint8_t *buffer, size_t size);
	size_t write(const char *str);
	size_t write(const char *buffer, size_t size);

	size_t print(const char *s);
	size_t print(char c);
	size_t print(unsigned char c);
	size_t print(int n);
	size_t print(unsigned int n);
	size_t print(long n);
	size_t print(unsigned long n);
	size_t print(double n, int digits = 2);
	size_t print(const String &s);

	size_t println(void);
	size_t println(const char *s);
	size_t println(const String &s);
	size_t println(int n);
};

#endif /* ROBOTDRIVERS_OLEDDRIVERS_ARDUINOCOMPAT_PRINT_H_ */
