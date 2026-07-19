/*
 * Print.cpp
 */
#include "Print.h"
#include <cstdio>
#include <cstring>

size_t Print::write(const uint8_t *buffer, size_t size)
{
	size_t n = 0;
	for (size_t i = 0; i < size; i++)
	{
		n += write(buffer[i]);
	}
	return n;
}

size_t Print::write(const char *str)
{
	if (str == nullptr)
	{
		return 0;
	}
	return write((const uint8_t *)str, strlen(str));
}

size_t Print::write(const char *buffer, size_t size)
{
	return write((const uint8_t *)buffer, size);
}

size_t Print::print(const char *s)
{
	return write(s);
}

size_t Print::print(char c)
{
	return write((uint8_t)c);
}

size_t Print::print(unsigned char c)
{
	return print((int)c);
}

size_t Print::print(int n)
{
	char buf[12];
	int len = snprintf(buf, sizeof(buf), "%d", n);
	return write(buf, (size_t)len);
}

size_t Print::print(unsigned int n)
{
	char buf[11];
	int len = snprintf(buf, sizeof(buf), "%u", n);
	return write(buf, (size_t)len);
}

size_t Print::print(long n)
{
	char buf[21];
	int len = snprintf(buf, sizeof(buf), "%ld", n);
	return write(buf, (size_t)len);
}

size_t Print::print(unsigned long n)
{
	char buf[21];
	int len = snprintf(buf, sizeof(buf), "%lu", n);
	return write(buf, (size_t)len);
}

size_t Print::print(double n, int digits)
{
	char buf[32];
	int len = snprintf(buf, sizeof(buf), "%.*f", digits, n);
	return write(buf, (size_t)len);
}

size_t Print::print(const String &s)
{
	return write((const uint8_t *)s.c_str(), s.length());
}

size_t Print::println(void)
{
	return write((const uint8_t *)"\r\n", 2);
}

size_t Print::println(const char *s)
{
	size_t n = print(s);
	n += println();
	return n;
}

size_t Print::println(const String &s)
{
	size_t n = print(s);
	n += println();
	return n;
}

size_t Print::println(int n)
{
	size_t r = print(n);
	r += println();
	return r;
}
