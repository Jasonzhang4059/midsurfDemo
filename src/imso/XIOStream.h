
#if !defined(__XIOSTREAM_H_INCLUDED__)
#define __XIOSTREAM_H_INCLUDED__ 
#pragma warning(disable:4996)
#include "varray.h"
#include "Str.h"
#include <vector>
#include <stdexcept>
# define _WINDOWS 1
using std::vector;
namespace base {

// the following functions convert the string to wstring or convert wstring to string
enum EncodingType {ANSI,UTF16_LE,UTF16_BE,UTF32_LE,UTF32_BE,UTF_8};

// convert to ANSI
// std::string toANSIstring(const wchar_t* pStr, unsigned int len = -1, int oriencodingtype = UTF16_LE); 

// convert to UTF_16 LE
// std::wstring toUnicodeString(const char* pStr, unsigned int len = -1, int oriencodingtype = ANSI);

class BaseIOStream;
class IOException : public std::runtime_error
{
public:
#ifdef _UNICODE
	IOException(const _TCHAR* msg) : std::runtime_error(toANSIstring(msg)) {}
#else
	IOException(const _TCHAR* msg) : std::runtime_error(msg) {}
#endif
	
};

//  Data Packager
template <class T>
struct DataPackager
{
	T* vBuffer;
	DataPackager(T& x) : vBuffer(&x) {}
    DataPackager(const T& x) : vBuffer(const_cast<T*>(&x)) {}
};

template <class T>
DataPackager<T> DataPack(T& x){return DataPackager<T>(x);}
template <class T>
const DataPackager<T> DataPack(const T& x){return DataPackager<T>(x);}

//  Array Packager
/////////////////////////////////////////////////////////////////////////////

template <class T>
struct ArrayPackager 
{
	typedef	T*	iterator;

	iterator	vStart,vEnd;
	ArrayPackager(T* a,T* b) : vStart(a),vEnd(b) {}
};

template <class T>
ArrayPackager<T> ArrayPack(T* a,T* b){return ArrayPackager<T>(a,b);}

//  Pointer Array Packager
/////////////////////////////////////////////////////////////////////////////

template <class T>
struct PointerArrayPackager 
{
	typedef	T**	iterator;

	iterator	vStart,vEnd;
	PointerArrayPackager(T** a,T** b) : vStart(a),vEnd(b) {}
	PointerArrayPackager(T*const* a,T*const* b)
		: vStart((T**)a),vEnd((T**)b) {}

	size_t	size() {
		return vEnd - vStart;
	}
	bool	empty() {
		return (vEnd == vStart);
	}
};

template <class T>
PointerArrayPackager<T> PointerArrayPack(T** a,T** b){return PointerArrayPackager<T>(a,b);}
template <class T>
PointerArrayPackager<T> PointerArrayPack(T*const* a,T*const* b){return PointerArrayPackager<T>(a,b);}

//  Direct Array Packager
/////////////////////////////////////////////////////////////////////////////

template <class T>
struct DirectArrayPackager
{
	T *vStart,*vEnd;
	DirectArrayPackager(T* a,T* b) : vStart(a),vEnd(b) {}
	DirectArrayPackager(const T* a, const T* b) : vStart(const_cast<T*>(a)),vEnd(const_cast<T*>(b)) {}
};

template <class T>
DirectArrayPackager<T> DirectPack(T* a,T* b){return DirectArrayPackager<T>(a,b);}

////////////////////////////////////////////////////
class  BaseOStream
{
public:
	BaseOStream(bool asciimode=false, bool buffered=false);
	~BaseOStream();

	bool ascii() {return m_asc;}
	void set_ascii(bool asciimode) {m_asc = asciimode;}

	
	bool buffered();						
	void set_buffered(bool buffered = true);
	unsigned long long get_buffer_capacity();				
	void set_buffer_capacity(unsigned long long capacity);

	virtual void flush();
	virtual bool fail() {return m_bfail;}
	virtual void textout(const _TCHAR* buf);
	virtual bool write(const void* buf, unsigned int sz);
	virtual BaseOStream& operator<<(const char x);		//charオペレータ
	//virtual BaseOStream& operator<<(const _TCHAR x);		//charオペレータ
	virtual BaseOStream& operator<<(const int x);		//intオペレータ
	virtual BaseOStream& operator<<(const long x);		//longオペレータ
	virtual BaseOStream& operator<<(const unsigned int x);		//unsigned intオペレータ
	virtual BaseOStream& operator<<(const unsigned long x);		//unsigned longオペレータ
	virtual BaseOStream& operator<<(const double x);	//doubleオペレータ
	virtual BaseOStream& operator<<(const float x);		//floatオペレータ
	virtual BaseOStream& operator<<(const _TCHAR* str);	//文字列オペレータ
	virtual BaseOStream& operator<<(bool x);	//文字列オペレータ
	virtual BaseOStream& operator<<(unsigned long long x);
    virtual BaseOStream& operator<<(String& x);

protected:	
	virtual bool write_buffer(const char* buffer, size_t sz)=0;
	virtual void throw_exception(const _TCHAR* msg)
	{
		m_bfail = true;
		throw IOException(msg);
	}
protected:
	bool m_asc;
	bool m_bfail;
	bool m_buffered;
	char* m_buffer;
	unsigned long long m_capacity;
	unsigned long long m_position;
};

////////////////////////////////////////////////////
class  BaseIStream 
{
public:	
	BaseIStream(bool asciimode=false);
    ~BaseIStream();

	bool ascii() {return m_asc;} 
	void set_ascii(bool asciimode) {m_asc = asciimode;}
	void set_delimitor(const _TCHAR* del)
	{
		int i = 0;
		for (i = 0; del[i] != _T('\0') && i < 511; i++) m_delimitor[i] = del[i];
		m_delimitor[i] = _T('\0');
	}

	virtual bool eof() {if (is_rbuffer_empty() && m_beof) return true; return false;}    
	virtual bool fail() {return m_bfail;}	
	virtual bool get_token(_TCHAR* buf, unsigned int sz);
	virtual int  read(void* buf, unsigned int sz);

	virtual BaseIStream& operator>>(char& x);	//charオペレータ
	//virtual BaseIStream& operator>>(_TCHAR& x);	//charオペレータ
	virtual BaseIStream& operator>>(int& x);	//intオペレータ
	virtual BaseIStream& operator>>(long& x);	//longオペレータ
	virtual BaseIStream& operator>>(unsigned int& x);	//unsigned intオペレータ
	virtual BaseIStream& operator>>(unsigned long& x);	//unsigned longオペレータ
	virtual BaseIStream& operator>>(double& x);	//doubleオペレータ
	virtual BaseIStream& operator>>(float& x);	//floatオペレータ
	virtual BaseIStream& operator>>(bool& x);	//boolオペレータ
	//virtual BaseIStream& operator>>(char& x);	//boolオペレータ
	virtual BaseIStream& operator>>(String& x);

protected:
	virtual char* read_to_buffer(char* buffer, size_t sz)=0;
	bool is_delimitor(_TCHAR c);
	virtual bool is_rbuffer_empty() {return (m_bufptr == m_bufend);}
	virtual void throw_exception(const _TCHAR* msg)
	{
		m_bfail = true;
		throw IOException(msg);
	}
	virtual void set_eof(bool e=true) {	m_beof = e;}
	virtual void initialize() 
	{
		m_bfail = false;
		m_beof = false;
		m_buf[0] = _T('\0');        // here will fill two zeros when unicode!!!
		m_bufptr = &m_buf[0];
		m_bufend = &m_buf[0];
	}

protected:
	bool m_beof;
	bool m_bfail;
	bool m_asc;
	_TCHAR m_delimitor[512];
	char m_buf[4096];
	char* m_bufptr;
	char* m_bufend;

private:
	BaseIStream(const BaseIStream&);
	//BaseIStream& operator=(const BaseIStream&);
};

////////////////////////////////////////////////////
class  BaseIOStream : public BaseIStream, public BaseOStream
{
public:
	BaseIOStream(bool asciimode=false, bool bufferedout=true)
		: BaseIStream(asciimode),BaseOStream(asciimode, bufferedout) {}
	bool ascii() {return BaseIStream::m_asc;}
	void set_ascii(bool asciimode) 
	{
		BaseIStream::set_ascii(asciimode);
		BaseOStream::set_ascii(asciimode);
	}
	virtual bool fail() {return BaseIStream::fail() && BaseOStream::fail();}
	
protected:
	virtual void throw_exception(const _TCHAR* msg)
	{
		BaseIStream::m_bfail = true;
		BaseOStream::m_bfail = true;
		throw IOException(msg);
	}
	virtual void initialize()
	{
		BaseIStream::m_bfail = false;
		BaseOStream::m_bfail = false;
		BaseIStream::initialize();
	}
};



template<class T>
inline BaseOStream& operator<<(BaseOStream& w,DataPackager<T> src) {
	w.write( src.vBuffer ,sizeof(T));
	return w;
}

template<class T>
inline BaseOStream& operator<<(BaseOStream& w,ArrayPackager<T> src) {
	for(ArrayPackager<T>::iterator i=src.vStart; i!=src.vEnd; i++ ) {
		w << *i;
	}
	return w;
}

template<class T>
inline BaseOStream& operator<<(BaseOStream& w,DirectArrayPackager<T> src) {
	w.write( src.vStart ,(unsigned int)(reinterpret_cast<unsigned long_PTR>(src.vEnd) - reinterpret_cast<unsigned long_PTR>(src.vStart)));
	return w;
}

template<class T>
inline BaseOStream& operator<<(BaseOStream& w,vector<T> &src) {
	DWORD dsize = src.size();
	for(std::vector<T>::const_iterator i=src.begin(); i!=src.end(); i++) {
		w << *i;
	}
	return w;
}

template<class T>
inline BaseOStream& operator<<(BaseOStream& w,const std::vector<T> &src) {
	DWORD dsize = src.size();
	for(std::vector<T>::const_iterator i=src.begin(); i!=src.end(); i++) {
		w << *i;
	}
	return w;
}

inline BaseOStream& operator<<(BaseOStream &w, BaseOStream& (* _f)(BaseOStream&)) {
	(*_f)(w);
	return w;
}

inline BaseOStream& endl(BaseOStream& w) { w << _T("\n"); return w; }

template<class T>
inline BaseIStream& operator>>(BaseIStream& c, DataPackager<T> packet) {
	c.read(packet.vBuffer, sizeof(T) );
	return c;
}

template<class T>
inline BaseIStream& operator>>(BaseIStream& c,ArrayPackager<T> &dst) {
	for(ArrayPackager<T>::iterator i=dst.vStart; i!=dst.vEnd; i++ ) {
		c >> *i;
	}
	return c;
}

template<class T>
inline BaseIStream& operator>>(BaseIStream& c,DirectArrayPackager<T> dst) {
	c.read( dst.vStart ,(unsigned int)(reinterpret_cast<unsigned long_PTR>(dst.vEnd)-reinterpret_cast<unsigned long_PTR>(dst.vStart)) );
	return c;
}

template<class T>
inline BaseIStream& operator>>(BaseIStream& c,std::vector<T> &dst) {
	for(std::vector<T>::iterator i=dst.begin(); i!=dst.end(); i++) {
		c >> *i;
	}
	return c;
}

///////////////////////////////////////////////////
class  MemoryStream : public BaseIOStream
{
public:
	MemoryStream(size_t allocate=4096);
	~MemoryStream();
	char* get_buffer();
	bool empty();
	void clear();
	size_t get_position();
	void set_position(size_t aPosition);
	size_t size() const;	
	void resize(size_t aSize);
	size_t capacity();
	void reserve(size_t aNewCapacity);
	void copy_to(BaseOStream& aStream, size_t aSize);
	void copy_all(BaseOStream& aStream) const;
	void copy_from(BaseIStream& aStream, size_t aSize);
	MemoryStream& operator=(const MemoryStream& a) 
	{
		clear();
		m_buffer = a.m_buffer;
		m_position = a.m_position;
		return *this;
	}

protected:
	virtual bool write_buffer(const char* buffer, size_t sz);
	virtual char* read_to_buffer(char* buffer, size_t sz);

private:
	varray<char>  m_buffer;
	size_t m_position; 
};

class  DebugStream : public BaseOStream
{
public:
	DebugStream() {set_ascii(true);};
protected:
	virtual bool write_buffer(const char* buffer, size_t sz);
};

extern DebugStream _dump;

//////////////////////////////////////////////////////
class  FileStream : public BaseIOStream
{
public:
	enum DSFILEIO_OPTION {
		TEXT   = 0x0001,
		BINARY = 0x0002,
		READ   = 0x0004,
		WRITE  = 0x0008
	};
	FileStream();
	FileStream(const _TCHAR* filename, unsigned int opt);
	~FileStream();
	bool open(const _TCHAR* filename, unsigned int opt);
	void close();
	unsigned long long size();

#if defined(_WINDOWS)
	FILE* get_file() { return m_fp; }
#else
	HANDLE get_file() { return m_file.m_hFile; }
#endif

protected:
	//Write
	virtual bool write_buffer(const char* buffer, size_t sz);
	virtual char* read_to_buffer(char* buffer, size_t sz);

private:
#if defined(_WINDOWS)
	
	FILE*   m_fp; //File Pointer
#else
	CFile	m_file;
#endif
	bool    m_bFailed;
};
}
#endif
