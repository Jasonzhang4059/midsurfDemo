//BaseStream
// 依存： stdafx.h, definition.h, basestream.h
#include "XIOStream.h"
#include <assert.h>
#include <algorithm>
#include <Windows.h>
#include <wchar.h>
#include <iostream>
#include <io.h>
# define _WINDOWS 1

namespace base
{

#define DEFAULT_READBUF_SIZE 1024

	//std::string toANSIstring(const wchar_t* pStr, unsigned int len/* = -1*/, int oriencodingtype /*= UTF16_LE*/)
	//{
	//	assert( pStr ); 
	//	assert( len >= 0 || len == -1); 
	//
	//	std::string buf ;
	//	switch(oriencodingtype) // the input string encoding type.
	//	{
	//	case UTF16_LE: 
	//		{
	//			// figure out how many narrow characters we are going to get 
	//			int nChars = WideCharToMultichar( CP_ACP , 0 , 
	//				pStr , len , NULL , 0 , NULL , NULL ) ; 
	//			
	//			if ( len == -1 ) // if the string is null-terminated.
	//				-- nChars ;
	//			if ( nChars == 0 )
	//				return "" ;
	//
	//			buf.resize( nChars ) ;
	//			WideCharToMultichar( CP_ACP , 0 , pStr , len , 
	//				const_cast<char*>(buf.c_str()) , nChars , NULL , NULL ) ; 
	//			break;
	//		}
	//	case UTF_8:
	//		{
	//			assert(false); //
	//			break;
	//		}
	//	default:
	//		assert(false);
	//	}
	//	return buf ; 
	//};

	// convert to UTF_16 LE
	//std::wstring toUnicodeString(const char* pStr, unsigned int len /*= -1*/, int oriencodingtype /*= ANSI*/)
	//{
	//	assert( pStr ); 
	//	assert( len >= 0 || len == -1); 
	//
	//	std::wstring buf ;
	//	switch(oriencodingtype) 
	//	{
	//	case ANSI:
	//		{
	//			// figure out how many wide characters we are going to get 
	//			int nChars = MulticharToWideChar( CP_ACP , 0 , pStr , len , NULL , 0 ) ; 
	//			if ( len == -1 )// if the string is null-terminated.
	//				-- nChars ; 
	//			if ( nChars == 0 )
	//				return L"" ;
	//
	//			buf.resize(nChars) ; 
	//			MulticharToWideChar( CP_ACP , 0 , pStr , len , 
	//				const_cast<wchar_t*>(buf.c_str()) , nChars ) ; 
	//
	//			break;
	//		}
	//	case UTF_8:
	//		{
	//			assert(false);
	//			break;
	//		}
	//	default:
	//		assert(false);
	//	}
	//	return buf ;
	//
	//};

	//////////////////////////////////////////////////////////
	// class BaseOStreamの実装
	BaseOStream::BaseOStream(bool asciimode, bool buffered)
	{
		m_asc = asciimode;
		m_buffered = buffered;
		m_capacity = 512;
		m_position = 0;
		m_bfail = false;

		if (m_buffered) {
			m_buffer = new char[(size_t)m_capacity];
		}
		else {
			m_buffer = NULL;
		}
	}

	BaseOStream::~BaseOStream()
	{
		if (m_buffer != NULL) {
			delete[] m_buffer;
		}
	}

	bool BaseOStream::buffered()
	{
		return m_buffered;
	}

	void BaseOStream::set_buffered(bool buffered)
	{
		m_buffered = buffered;
		if (buffered) {
			if (m_buffer == NULL)
				m_buffer = new char[(size_t)m_capacity];
		}
		else {
			if (m_buffer)
				delete m_buffer;
			m_buffer = NULL;
		}
	}

	unsigned long long BaseOStream::get_buffer_capacity()
	{
		return m_capacity;
	}

	void BaseOStream::set_buffer_capacity(unsigned long long capacity)
	{
		if (m_capacity < capacity) {
			char* newBuffer = new char[(size_t)capacity];
			::memcpy(newBuffer, m_buffer, (size_t)m_position);
			delete m_buffer;
			m_buffer = newBuffer;
			m_capacity = capacity;
		}
	}

	void BaseOStream::flush()
	{
		if (m_position == 0) return;
		if (m_buffer == NULL) return;
		if (!write_buffer(m_buffer, (size_t)m_position)) {
			throw_exception(_T("write error (flush)"));
		}
		m_position = 0;
	}

	void BaseOStream::textout(const _TCHAR* buf)
	{
		if (m_buffered) {
			unsigned int len = (unsigned int)_tcslen(buf);
			if (m_capacity < m_position + len) {
				//flush();
				if (!write_buffer(m_buffer, (size_t)m_position)) {
					throw_exception(_T("write error (flush)"));
				}
				m_position = 0;
			}
			::memcpy(m_buffer + m_position, buf, len * sizeof(_TCHAR));
			m_position += len;
		}
		else {
			if (!write_buffer((const char*)buf, _tcslen(buf) * sizeof(_TCHAR)))
				throw_exception(_T("write error(textout)"));
		}
	}

	bool BaseOStream::write(const void* tbuf, unsigned int sz)
	{
		const char* buf = (const char*)tbuf;
		if (m_buffered) {
			if (m_capacity < m_position + sz) {
				if (!write_buffer(m_buffer, (size_t)m_position)) {
					m_position = 0;
					throw_exception(_T("write error(write)"));

					return false;
				}
				m_position = 0;
			}
			::memcpy(m_buffer + m_position, buf, sz);
			m_position += sz;
			return true;
		}
		else {
			if (!write_buffer(buf, sz)) {
				throw_exception(_T("write error(write)"));
				return false;
			}
			return true;
		}
	}

	BaseOStream& BaseOStream::operator<<(const char x)
	{
		if (ascii()) {
			_TCHAR str[256];
			_stprintf(str, _T("%x"), x);
			textout(str);
		}
		else {
			write((char*)&x, sizeof(char));
		}
		return *this;
	}

	//BaseOStream& BaseOStream::operator<<(const _TCHAR x)
	//{
	//	if (ascii()) {
	//		_TCHAR str[256];
	//		_stprintf(str, _T("%c"), x);
	//		textout(str);
	//	} else {
	//		write((char*)&x, sizeof(_TCHAR));
	//	}
	//	return *this;
	//}

	BaseOStream& BaseOStream::operator<<(const int x)
	{
		if (ascii()) {
			_TCHAR str[256];
			_stprintf(str, _T("%d"), x);
			textout(str);
		}
		else {
			write((char*)&x, sizeof(int));
		}
		return *this;
	}

	BaseOStream& BaseOStream::operator<<(const long x)
	{
		if (ascii()) {
			_TCHAR str[256];
			_stprintf(str, _T("%ld"), x);
			textout(str);
		}
		else {
			write((char*)&x, sizeof(long));
		}
		return *this;
	}

	BaseOStream& BaseOStream::operator<<(const unsigned int x)
	{
		if (ascii()) {
			_TCHAR str[256];
			_stprintf(str, _T("%u"), x);
			textout(str);
		}
		else {
			write((char*)&x, sizeof(unsigned int));
		}
		return *this;
	}

	BaseOStream& BaseOStream::operator<<(const unsigned long x)
	{
		if (ascii()) {
			_TCHAR str[256];
			_stprintf(str, _T("%lu"), x);
			textout(str);
		}
		else {
			write((char*)&x, sizeof(unsigned long));
		}
		return *this;
	}

	BaseOStream& BaseOStream::operator<<(const double x)
	{
		if (ascii()) {
			_TCHAR str[256];
			_stprintf(str, _T("%.17lf"), x);
			textout(str);
		}
		else {
			write((char*)&x, sizeof(double));
		}
		return *this;
	}

	BaseOStream& BaseOStream::operator<<(const float x)
	{
		if (ascii()) {
			_TCHAR str[256];
			_stprintf(str, _T("%f"), x);
			textout(str);
		}
		else {
			write((char*)&x, sizeof(float));
		}
		return *this;
	}

	BaseOStream& BaseOStream::operator<<(bool x)
	{
		if (ascii()) {
			_TCHAR str[256];
			_stprintf(str, _T("%d"), x);
			textout(str);
		}
		else {
			write((char*)&x, sizeof(bool));
		}
		return *this;
	}

	BaseOStream& BaseOStream::operator<<(const _TCHAR* str)
	{
		if (ascii()) {
			textout(str);
		}
		else {
			write((char*)str, (unsigned int)(_tcslen(str) * sizeof(_TCHAR)));
		}
		return *this;
	}

	BaseOStream& BaseOStream::operator<<(unsigned long long x)
	{
		if (ascii()) {
			_TCHAR str[256];
			_stprintf(str, _T("%I64u"), x);
			textout(str);
		}
		else {
			write((char*)&x, sizeof(unsigned long long));
		}
		return *this;
	}
	BaseOStream& BaseOStream::operator<<(String& x)
	{
		/*if (ascii()) {
			String str[256];
			_stprintf(str, _T("%I64u"), x);
			textout(str);
		} else {
			write((char*)&x, sizeof(unsigned long long));
		}*/
		return *this;
	}

	//////////////////////////////////////////////////////////
	// class BaseIStreamの実装
	BaseIStream::BaseIStream(bool asciimode) {
		m_asc = asciimode;
		_tcscpy(m_delimitor, _T(" \t\n\0"));
		initialize();
	}
	BaseIStream::~BaseIStream()
	{

	}
	bool BaseIStream::get_token(_TCHAR* buf, unsigned int sz)
	{
		int   c = 0;
		_TCHAR* d = buf;

		buf[0] = _T('\0');

		while (true) {
			if (is_rbuffer_empty()) {
				m_bufend = m_bufptr = m_buf;
				m_buf[0] = '\0';
				m_bufend = read_to_buffer(m_buf, 4096);
				if (m_bufend == NULL) {
					initialize();
					throw_exception(_T("read error(BaseIStream::get_token)"));
					return false;
				}
				if (unsigned int(m_bufend - m_bufptr) < 4096) set_eof();
			}
			if (!is_delimitor(*((_TCHAR*)m_bufptr)) || *((TCHAR*)m_bufptr) == _T('\0')) break;
			if (eof()) break;
			m_bufptr += sizeof(_TCHAR);
		}

		while (true) {
			if (is_rbuffer_empty()) {
				m_bufend = m_bufptr = m_buf;
				m_buf[0] = '\0';
				m_bufend = read_to_buffer(m_buf, 4096);
				if (m_bufend == NULL) {
					initialize();
					throw_exception(_T("read error(BaseIStream::get_token)"));
					return false;
				}
				if (unsigned int(m_bufend - m_bufptr) < 4096) set_eof();
			}
			if (is_delimitor(*((_TCHAR*)m_bufptr)) || *((TCHAR*)m_bufptr) == _T('\0')) break;
			if (c + sizeof(_TCHAR) > sz) break;
			if (eof()) break;

			*((_TCHAR*)d) = *((_TCHAR*)m_bufptr);
			d += sizeof(_TCHAR);
			m_bufptr += sizeof(_TCHAR);
			c += sizeof(_TCHAR);
		}
		*d = _T('\0');

		if (m_bufptr != m_bufend) {
			(*(_TCHAR**)&m_bufptr)++;
		}

		int n = (int)_tcslen(buf);
		for (int i = 0; i < n; i++)
		{
			if (is_delimitor(_T('\n')))
			{
				if (is_delimitor(buf[i])) buf[i] = _T('\0');
			}
			else
			{
				if (is_delimitor(buf[i]) || buf[i] == _T('\r')) buf[i] = _T('\0');
			}
		}

		return true;
	}

	int BaseIStream::read(void* tbuf, unsigned int size)
	{
		char* buf = (char*)tbuf;
		unsigned int rest = size;

		while (rest > 0) {
			unsigned int bufcap = (unsigned int)(m_bufend - m_bufptr);
			if (static_cast<int>(bufcap) <= 0) {
				m_bufptr = m_bufend = m_buf;
				m_bufend = read_to_buffer(m_buf, 4096);
				if (m_bufend == NULL) {
					initialize();
					throw_exception(_T("read error(BaseIStream::read)"));
					return 0;
				}

				size_t trest = m_bufend - m_bufptr;
				if (trest < 4096) set_eof();

				bufcap = (unsigned int)(m_bufend - m_bufptr);
			}
			size_t need_to_copy = min(rest, bufcap);
			::memcpy(buf, m_bufptr, need_to_copy);

			m_bufptr += need_to_copy;
			buf += need_to_copy;
			rest -= (unsigned int)need_to_copy;

			if (eof()) break;
		}
		return size - rest;
	}

	BaseIStream& BaseIStream::operator>>(char& x)
	{
		if (ascii()) {
			_TCHAR buf[DEFAULT_READBUF_SIZE];
			if (get_token(buf, DEFAULT_READBUF_SIZE)) {
				_stscanf(buf, _T("%x"), &x);
			}
		}
		else {
			read(&x, sizeof(char));
		}
		return *this;
	}
	//BaseIStream& BaseIStream::operator>>(char& x)
	//{
	//	if (ascii()) {
	//		_TCHAR buf[DEFAULT_READBUF_SIZE];
	//		if (get_token(buf, DEFAULT_READBUF_SIZE)) {
	//			_stscanf(buf, _T("%c"), &x);
	//		}
	//	} else {
	//		read(&x, sizeof(char));
	//	}
	//	return *this;
	//}
	//BaseIStream& BaseIStream::operator>>(_TCHAR& x)
	//{
	//	if (ascii()) {
	//		_TCHAR buf[DEFAULT_READBUF_SIZE];
	//		if (get_token(buf, DEFAULT_READBUF_SIZE)) {
	//			_stscanf(buf, _T("%c"), &x);
	//		}
	//	} else {
	//		read((char*)&x, sizeof(_TCHAR));
	//	}
	//	return *this;
	//}

	BaseIStream& BaseIStream::operator>>(int& x)
	{
		if (ascii()) {
			_TCHAR buf[DEFAULT_READBUF_SIZE];
			if (get_token(buf, DEFAULT_READBUF_SIZE)) {
				_stscanf(buf, _T("%d"), &x);
			}
		}
		else {
			read((char*)&x, sizeof(int));
		}
		return *this;
	}

	BaseIStream& BaseIStream::operator>>(long& x)
	{
		if (ascii()) {
			_TCHAR buf[DEFAULT_READBUF_SIZE];
			if (get_token(buf, DEFAULT_READBUF_SIZE)) {
				_stscanf(buf, _T("%ld"), &x);
			}
		}
		else {
			read((char*)&x, sizeof(long));
		}
		return *this;
	}

	BaseIStream& BaseIStream::operator>>(unsigned int& x)
	{
		if (ascii()) {
			_TCHAR buf[DEFAULT_READBUF_SIZE];
			if (get_token(buf, DEFAULT_READBUF_SIZE)) {
				_stscanf(buf, _T("%u"), &x);
			}
		}
		else {
			read((char*)&x, sizeof(unsigned int));
		}
		return *this;
	}

	BaseIStream& BaseIStream::operator>>(unsigned long& x)
	{
		if (ascii()) {
			_TCHAR buf[DEFAULT_READBUF_SIZE];
			if (get_token(buf, DEFAULT_READBUF_SIZE)) {
				_stscanf(buf, _T("%lu"), &x);
			}
		}
		else {
			read((char*)&x, sizeof(unsigned long));
		}
		return *this;
	}

	BaseIStream& BaseIStream::operator>>(double& x)
	{
		if (ascii()) {
			_TCHAR buf[DEFAULT_READBUF_SIZE];
			if (get_token(buf, DEFAULT_READBUF_SIZE)) {
				_stscanf(buf, _T("%lf"), &x);
			}
		}
		else {
			read((char*)&x, sizeof(double));
		}
		return *this;
	}

	BaseIStream& BaseIStream::operator>>(float& x)
	{
		if (ascii()) {
			_TCHAR buf[DEFAULT_READBUF_SIZE];
			if (get_token(buf, DEFAULT_READBUF_SIZE)) {
				_stscanf(buf, _T("%f"), &x);
			}
		}
		else {
			read((char*)&x, sizeof(float));
		}
		return *this;
	}

	BaseIStream& BaseIStream::operator>>(bool& x)
	{
		if (ascii()) {
			_TCHAR buf[DEFAULT_READBUF_SIZE];
			if (get_token(buf, DEFAULT_READBUF_SIZE)) {
				int t;
				_stscanf(buf, _T("%d"), &t);
				x = t ? true : false;
			}
		}
		else {
			read((char*)&x, sizeof(bool));
		}
		return *this;
	}
	BaseIStream& BaseIStream::operator>>(String& x)
	{
		/*if (ascii()) {
			String buf[DEFAULT_READBUF_SIZE];
			if (get_token(buf, DEFAULT_READBUF_SIZE)) {
				int t;
				_stscanf(buf, _T("%d"), &t);
				x = t ? true : false;
			}
		} else {
			read((char*)&x, sizeof(bool));
		}*/
		return *this;
	}
	bool BaseIStream::is_delimitor(_TCHAR c)
	{
		_TCHAR *p = std::find(&m_delimitor[0], &m_delimitor[_tcslen(m_delimitor)], c);
		if (p == &m_delimitor[_tcslen(m_delimitor)]) return false;
		return true;
	}

	MemoryStream::MemoryStream(size_t aBufferCapacity)
		: BaseIOStream(false, false)
	{
		m_buffer.reserve(aBufferCapacity);
		m_position = 0;
	}

	MemoryStream::~MemoryStream()
	{
	}

	char* MemoryStream::get_buffer()
	{
		return (char*)m_buffer.begin();
	}

	bool MemoryStream::empty()
	{
		return (m_buffer.size() == 0);
	}

	void MemoryStream::clear()
	{
		m_position = 0;
		m_buffer.clear();
	}

	size_t MemoryStream::get_position()
	{
		return m_position;
	}

	void MemoryStream::set_position(size_t aPosition)
	{
		m_position = aPosition;
	}

	size_t MemoryStream::size()	const
	{
		return m_buffer.size() - m_position;
	}

	void MemoryStream::resize(size_t aSize)
	{
		m_buffer.resize(aSize + m_position);
	}

	size_t MemoryStream::capacity()
	{
		return m_buffer.capacity();
	}

	void MemoryStream::reserve(size_t aNewCapacity)
	{
		m_buffer.reserve(aNewCapacity);
	}

	//Write
	bool MemoryStream::write_buffer(const char* aData, size_t aSize)
	{
		if (aSize > 0) {
			size_t presize = m_buffer.size();
			m_buffer.resize(presize + aSize);
			::memcpy(&m_buffer[presize], aData, aSize);
		}

		return true;
	}

	void MemoryStream::copy_from(BaseIStream& aStream, size_t aSize)
	{
		if (aSize <= 0) return;
		size_t presize = m_buffer.size();
		m_buffer.resize(presize + aSize);
		aStream.read(&m_buffer[presize], static_cast<unsigned int>(aSize));
	}

	char* MemoryStream::read_to_buffer(char* aData, size_t aSize)
	{
		if (size() == 0) {
			return aData;
		}

		size_t sz = min(size(), aSize);
		::memcpy(aData, m_buffer.begin() + m_position, sz);
		m_position += sz;

		return (aData + sz);
	}

	void MemoryStream::copy_to(BaseOStream& aStream, size_t aSize)
	{
		size_t sz = min(size(), aSize);
		aStream.write(m_buffer.begin() + m_position, static_cast<unsigned int>(sz));
		m_position += sz;
	}

	void MemoryStream::copy_all(BaseOStream& aStream) const
	{
		aStream.write(m_buffer.begin(), static_cast<unsigned int>(size()));
	}

	DebugStream _dump;

#if defined(_WINDOWS)
	bool DebugStream::write_buffer(const char* buffer, size_t sz) {
		assert(sz >= 0);
		//OutputDebugString((const _TCHAR*)buffer);
		return true;
	}

#else

	bool DebugStream::write_buffer(const char* buffer, size_t sz) {
		std::cerr << (const char*)buffer;
		return true;
	}

#endif

	FileStream::FileStream() : BaseIOStream(false, false)
	{
#if defined(_WINDOWS)
		//m_hFile = INVALID_HANDLE_VALUE;
#else
		m_fp = NULL;
#endif

	}

	FileStream::FileStream(const _TCHAR* filename, unsigned int opt) : BaseIOStream(false, false)
	{
		open(filename, opt);
	}

	FileStream::~FileStream()
	{
		close();
	}

	unsigned long long FileStream::size()
	{
#if defined(_WINDOWS)
		//return GetFileSize( m_hFile, NULL );
		return filelength(fileno(m_fp));
#else
		//return filelength( fileno(m_fp) );
		return m_file.GetLength();
#endif
	}

	bool FileStream::open(const _TCHAR* filename, unsigned int opt)
	{
#if defined(_WINDOWS)

		BaseIStream::initialize();
		int offset = 0;
		char str[256];

		//set_fail(false);
		if (opt&READ)  str[offset++] = 'r';
		if (opt&WRITE) str[offset++] = 'w';
		if (opt&TEXT) str[offset++] = 't';
		if (opt&BINARY) str[offset++] = 'b';
		str[offset++] = '\0';

		m_fp = ::fopen(filename, str);

		if (!m_fp) {
			throw_exception("file open error");
			return false;
		}
		return true;
#else
		BaseIStream::initialize();
		try
		{
			DWORD nOpenFlags = 0;
			if (opt&READ) { nOpenFlags |= CFile::modeRead | CFile::osSequentialScan;	nOpenFlags |= CFile::shareExclusive | CFile::typeBinary; }
			if (opt&WRITE) { nOpenFlags |= CFile::modeWrite;	nOpenFlags |= CFile::shareExclusive | CFile::modeCreate | CFile::typeBinary; }
			//m_file.Open(filename,nOpenFlags);
		}
		catch (CFileException *e)
		{
#ifdef _DEBUG
			afxDump << _T("File could not be opened ") << e->m_cause << _T("\n");
#endif
			e->Delete();
			return false;

		}
		return true;
#endif
	}

	void FileStream::close()
	{
#if defined(_WINDOWS)
		if (m_fp) fclose(m_fp);
		m_fp = NULL;
#else
		if (m_file.m_hFile != INVALID_HANDLE_VALUE)
			m_file.Close();
#endif
		initialize();
	}

	bool FileStream::write_buffer(const char* buffer, size_t sz)
	{
#if defined(_WINDOWS)
		if (!m_fp) {
			return false;
		}
		fwrite(buffer, sz, 1, m_fp);
		return true;
#else


		if (m_file.m_hFile == INVALID_HANDLE_VALUE) {
			return false;
		}
		try
		{
			m_file.Write(buffer, (unsigned int)sz);
		}
		catch (CFileException *e)
		{
#ifdef _DEBUG
			afxDump << _T("File could not be writed ") << e->m_cause << _T("\n");
#endif
			e->Delete();
			return false;
		}
		return true;
#endif
	}

	//bufferにsz分のデータを読み込む
	char* FileStream::read_to_buffer(char* buffer, size_t sz)
	{
#if defined(_WINDOWS)
		if (!m_fp) {
			return NULL;
		}

		char* pend = NULL;
		//処理系によって実装を変えてくれ
		int rsz = fread(buffer, 1, sz, m_fp);
		if (rsz < 1) {
			if (feof(m_fp))
				pend = &buffer[sz];
			else
				pend = NULL;
		}
		else {
			pend = &buffer[sz];
		}
		return pend;
#else


		if (m_file.m_hFile == INVALID_HANDLE_VALUE) {
			return NULL;
		}

		char* pend = NULL;
		unsigned int readed = 0;
		try
		{
			readed = m_file.Read(buffer, (unsigned int)sz);
		}
		catch (CFileException *e)
		{
#ifdef _DEBUG
			afxDump << _T("File could not be read ") << e->m_cause << _T("\n");
#endif
			e->Delete();
			return NULL;
		}


		if (readed < sz) {
			return &buffer[readed];
		}
		pend = &buffer[readed];
		return pend;
#endif
	}


};
