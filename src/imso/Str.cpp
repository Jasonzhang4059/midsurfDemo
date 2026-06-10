//ï∂éöóÒóp
// àÀë∂ÅF stdafx.h, definition.h, dsstring.h

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "XIOStream.h"
#include "str.h"

namespace base {
	
String::String() {
	m_buf = NULL;
	m_buf = new _TCHAR[1];
	m_buf[0] = _T('\0');
	m_allocsize = 1;
}
	
String::String(const _TCHAR* _format, ...)
{
	m_buf = NULL;
	
	_TCHAR buf[1024] ;
	va_list args ;
	va_start(args, _format) ;
	_vstprintf(buf, _format, args) ;
	va_end(args) ;
	
	int len = static_cast<int>(_tcslen(buf));
	
	m_buf = new _TCHAR[len+1];
	_tcscpy(m_buf, buf);
	m_allocsize = len+1;
}

String::String(const String& str) 
{
	m_buf = NULL;
	
	m_allocsize = static_cast<int>(_tcslen(str.m_buf) + 1);
	m_buf = new _TCHAR[m_allocsize];
	::_tcscpy(m_buf, str.m_buf);
}

String::String(int val)
{
	m_buf = NULL;
	
	m_buf = new _TCHAR[1];
	m_buf[0] = _T('\0');
	m_allocsize = 1;
	*this = val;
}

String::String(float  val)
{
	m_buf = NULL;
	
	m_buf = new _TCHAR[1];
	m_buf[0] = _T('\0');
	m_allocsize = 1;
	*this = val;
}

String::String(double val)
{
	m_buf = NULL;
	
	m_buf = new _TCHAR[1];
	m_buf[0] = _T('\0');
	m_allocsize = 1;
	*this = val;
}

String::String(_TCHAR c)
{
	m_buf = NULL;
	m_buf = new _TCHAR[2];
	m_buf[0] = c;
	m_buf[1] = _T('\0');
	m_allocsize = 2;
}

#ifdef _UNICODE
String::String(const std::wstring& str)
{
	m_buf = NULL;
	m_buf = new wchar_t[1];
	m_buf[0] = L'\0';
	m_allocsize = 1;

	*this = str.c_str();
}
#else
//String::String(const std::string& str)
//{
//	m_buf = NULL;
//	m_buf = new _TCHAR[1];
//	m_buf[0] = '\0';
//	m_allocsize = 1;
//
//	*this = str.c_str();
//}
#endif


String::~String() {
	if (m_buf) delete [] m_buf;
	m_allocsize = 0;
}

void String::clear() {
	if (m_buf) delete [] m_buf;
	m_buf = new _TCHAR[1];
	m_buf[0] = _T('\0');
	m_allocsize = 1;
}

_TCHAR* String::getbuf() const {
	return m_buf;
}

void String::reserve(int size) {
	if (m_buf) delete [] m_buf;
	m_allocsize = size;
	m_buf = new _TCHAR[m_allocsize];
	m_buf[0] = _T('\0');
}

int String::len() const {
	return static_cast<int>(::_tcslen(m_buf));
}

const _TCHAR& String::at(int i) const 
{
	return m_buf[i];
}

void String::set(int i, _TCHAR c)
{
	m_buf[i] = c;
}

String& String::format(const _TCHAR* _format, ...)
{
	_TCHAR buf[1024] ;
	
	va_list args ;
	va_start(args, _format) ;
	_vstprintf(buf, _format, args) ;
	va_end(args) ;
	
	*this = buf ;
	return *this ;
}

String& String::add_format(const _TCHAR* _format, ...)
{
	_TCHAR buf[1024] ;
	
	va_list args ;
	va_start(args, _format) ;
	_vstprintf(buf, _format, args) ;
	va_end(args) ;
	
	*this += buf ;
	return *this ;
}

int String::find(_TCHAR c, int offset) const
{
	int i;
	int find = 0;
	int l = len();
	for (i = 0; i < l; i++) {
		if (m_buf[i+offset] == c) {
			find = 1;
			break;
		}
	}
	
	if (find) return i;
	else return -1;
}

int String::find(const _TCHAR* str, int offset) const
{
	_TCHAR* c = ::_tcsstr(&m_buf[offset], str);
	
	if (c) {
		return static_cast<int>((c - m_buf) / sizeof(_TCHAR));
	} else {
		return -1;
	}
}

String String::replace(const _TCHAR from, const _TCHAR to, 
				   int all, int offset)
{
	String r = *this;
	int l = len();
	
	for (int i = 0; i < l; i++) {
		if (r.at(i+offset) == from) {
			r.set(i+offset, to);
			if (!all || to == '\0') break;
		}
	}
	
	return r;
}

String String::replace(const _TCHAR* from, const _TCHAR* to, 
				   int all, int offset)
{
	String r;
	int pos = offset;
	
	while (1) {
		int fpos = find(from, pos);
		if (fpos == -1) break;
		
		r += mid(pos, fpos-pos);
		r += to;
		pos = static_cast<int>(fpos + _tcslen(from));
		if (!all) break;
	}
	r += mid(pos, len()-pos);
	
	return r;
}

String String::delchar(const _TCHAR* c, int offset)
{
	_TCHAR* buf = new _TCHAR[len()+1];
	_TCHAR* d = buf;
	
	for (_TCHAR* s = &m_buf[offset]; *s != _T('\0'); s++) {
		int bfind=0;
		for (_TCHAR* f = (_TCHAR*)c; *f != _T('\0'); f++) {
			if (*f == *s) {
				bfind = 1;
				break;
			}
		}
		if (bfind) continue;
		*d++ = *s;
	}
	*d = _T('\0');
	String r(buf);
	delete [] buf;
	return r;
}

String String::delleftwhite(const _TCHAR* c)
{
	_TCHAR* s = m_buf;
	
	int bfind=1;
	while (bfind) {
		if (*s == _T('\0')) break;
		
		bfind = 0;
		for (_TCHAR* f = (_TCHAR*)c; *f != _T('\0'); f++) {
			if (*f == *s) {
				bfind = 1;
				break;
			}
		}
		if (bfind) s++;
	}
	
	String r(s);
	
	return r;
	
}

String String::left(int l) const
{
	return mid(0, l);
}

String String::right(int l) const
{
	return mid(len()-l, l);
}

String String::mid(int from, int l) const
{
	if (len() == 0) {
		return String(_T(""));
	} else {
		String r;
		for (int i = 0; i < l; i++) r += at(from + i);
		return r;
	}
}

bool String::equals(const String& a){
	return (*this == a)?true:false;
}

String String::sub(int from, int l) const
{
	String r;
	
	r += left(from);
	r += mid(from+l, len()-(from+l));
	
	return r;
}

String String::upper() const
{
	String r = *this;
	int l = r.len();
	
	for (int i = 0; i < l; i++) {
		_TCHAR c = r.at(i);
		if (_T('a') <= c && c <= _T('z')) {
			r.set(i, c-_T('a')+_T('A'));
		}
	}
	
	return r;
}

String String::lower() const
{
	String r = *this;
	int l = r.len();
	
	for (int i = 0; i < l; i++) {
		_TCHAR c = r.at(i);
		if (_T('A') <= c && c <= _T('Z')) {
			r.set(i, c-_T('A')+_T('a'));
		}
	}
	
	return r;
}


String String::getword(const _TCHAR* deml)
{
	int i, j;
	
	*this = delleftwhite(deml);
	if (this->len() <= 0) return String(_T(""));
	
	_TCHAR *buf = new _TCHAR[len()+1];
	_tcscpy(buf, this->getbuf());
	
	for (i = 0; i < static_cast<int>(_tcslen(buf)); i++) {
		if (buf[i] == _T('\0')) break;
		bool bFind = false;
		for (j = 0; j < static_cast<int>(_tcslen(deml)); j++) {
			if (buf[i] == deml[j]) {
				buf[i] = '\0';
				bFind = true;
				break;
			}
		}
		if (bFind) break;
	}
	
	if (this->len() != (int)_tcslen(buf)) {
		*this = sub(0, static_cast<int>(_tcslen(buf)));
		*this = delleftwhite(deml);
	} else {
		*this = _T("");
	}
	String result(buf);
	delete [] buf;
	return result;
}

int String::atoi() const
{
	return ::_ttoi(m_buf);
}

double String::atof() const
{
	return ::_tstof(m_buf);
}

int String::isint() const
{
	_TCHAR* c = m_buf;
	int ret = 1;
	
	while (*c != '\0') {
		if( !('0' <= *c && *c <= '9' || *c == '-' || *c == '+') ){
			ret = 0;
			break;
		}
		c++;
	}
	
	return ret;
}

int String::isfloat() const
{
	_TCHAR* c = m_buf;
	int ret = 1;
	
	while (*c != '\0') {
		if( !('0' <= *c && *c <= '9' || *c == '.' || *c == '-' || *c == '+') ){
			ret = 0;
			break;
		}
		c++;
	}
	
	return ret;
}

String::operator const _TCHAR*() const {
	return (const _TCHAR*)m_buf;
}

String& String::operator=(const _TCHAR* a) {
	delete [] m_buf;
	m_allocsize = static_cast<int>(_tcslen(a)+1);
	m_buf = new _TCHAR[m_allocsize];
	::_tcscpy(m_buf, a );
	
	return *this;
}

String& String::operator=(const String& a) {
	delete [] m_buf;
	m_allocsize = a.m_allocsize;
	m_buf = new _TCHAR[m_allocsize];
	::_tcscpy(m_buf, a.m_buf );
	
	return *this;
}

String& String::operator=(int   a)
{
	_TCHAR buf[256];
	_stprintf(buf, _T("%d"), a);
	*this = buf;
	
	return *this;
}

String& String::operator=(float a)
{
	_TCHAR buf[256];
	_stprintf(buf, _T("%f"), a);
	*this = buf;
	
	return *this;
}

String& String::operator=(double a)
{
	_TCHAR buf[256];
	_stprintf(buf, _T("%lf"), a);
	*this = buf;
	
	return *this;
}
#ifdef _UNICODE
String& String::operator=(const std::wstring& str)
{
	*this = str.c_str();

	return *this;
}
#else
//String& String::operator=(const std::string& str)
//{
//	*this = str.c_str();
//
//	return *this;
//}
#endif


String& String::operator+=(const _TCHAR* a) {
	m_allocsize = static_cast<int>(m_allocsize + ::_tcslen(a) + 1);
	_TCHAR* tmp = new _TCHAR[m_allocsize];
	::_tcscpy(tmp, m_buf );
	::_tcscat(tmp, a);
	delete [] m_buf;
	m_buf = tmp;
	
	return *this;
}

String& String::operator+=(const _TCHAR a) {
	m_allocsize = m_allocsize + 1;
	_TCHAR* tmp = new _TCHAR[m_allocsize];
	::_tcscpy(tmp, m_buf );
	
	int l = static_cast<int>(_tcslen(tmp));
	tmp[l] = a;
	tmp[l+1] = '\0';
	
	delete [] m_buf;
	m_buf = tmp;
	
	return *this;
}

String& String::operator+=(int   a)
{
	String x = a;
	*this += x;
	return *this;
}

String& String::operator+=(float a)
{
	String x = a;
	*this += x;
	return *this;
}

String& String::operator+=(double a)
{
	String x = a;
	*this += x;
	return *this;
}
#ifdef _UNICODE
String& String::operator+=(const std::wstring& str)
{
	*this += str.c_str();
	return *this;
}
#else
//String& String::operator+=(const std::string& str)
//{
//	*this += str.c_str();
//	return *this;
//}
#endif



int operator==(const String& a, const String& b)
{
	return (_tcscmp(a.m_buf, b.m_buf) == 0);
}

int operator==(const String& a, const _TCHAR* b)
{
	return (_tcscmp(a.m_buf, b) == 0);
}

int operator!=(const String& a, const String& b)
{
	return (_tcscmp(a.m_buf, b.m_buf) != 0);
}

int operator!=(const String& a, const _TCHAR* b)
{
	return (_tcscmp(a.m_buf, b) != 0);
}


String
	operator+(const String& a, const String& b)
{
	String ret = a;
	ret += b;
	
	return ret;
}

String 
	operator+(const String& a, const _TCHAR* b)
{
	String r = a;
	r += b;
	
	return r;
}

String
	operator+(const _TCHAR* b, const String& a)
{
	String r(b);
	r += a;
	
	return r;
}

String 
	operator+(const String& a, const _TCHAR b)
{
	String r = a;
	r += b;
	
	return r;
}

String 
	operator+(const _TCHAR b, const String& a)
{
	String r = a;
	r += b;
	
	return r;
}

String
	operator+(const String& a, const int& b)
{
	_TCHAR buf[256];
	_stprintf(buf, _T("%d"), b);
	String r = a;
	r = r + buf;
	
	return r;
}

String
	operator+(const String& a, const float& b)
{
	_TCHAR buf[256];
	_stprintf(buf, _T("%f"), b);
	String r = a;
	r = r + buf;
	
	return r;
}

String
	operator+(const String& a, const double& b)
{
	_TCHAR buf[256];
	_stprintf(buf, _T("%lf"), b);
	String r = a;
	r = r + buf;
	
	return r;
}
	
bool   operator>(const String& a, const String& b)
{
	int i;

	if (a.len() < b.len()) {
		for (i = 0; i < a.len(); i++) {
			if (a.at(i) > b.at(i)) return true;
			else if (a.at(i) < b.at(i)) return false;
		}
	} else {
		for (i = 0; i < b.len(); i++) {
			if (a.at(i) > b.at(i)) return true;
			else if (a.at(i) < b.at(i)) return false;
		}
	}
	return false;
}

bool   operator<(const String& a, const String& b)
{
	int i;

	if (a.len() < b.len()) {
		for (i = 0; i < a.len(); i++) {
			if (a.at(i) > b.at(i)) return false;
			else if (a.at(i) < b.at(i)) return true;
		}
	} else {
		for (i = 0; i < b.len(); i++) {
			if (a.at(i) > b.at(i)) return false;
			else if (a.at(i) < b.at(i)) return true;
		}
	}
	return false;
}

}