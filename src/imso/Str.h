//  Real-time Simulation Library
//	str.h: ストリングクラス宣言部
//
//  	Copyright(C) 2001,  T. Ishii
//////////////////////////////////////////////////////////////////////

#ifndef __STRING_H_INCLUDED__141
#define __STRING_H_INCLUDED__141
#include <tchar.h>

namespace base 
{

//String
// 文字列を扱うためのクラス
class  String {
//オペレーション
public:
	
	String();//デフォルトコンストラクタ
	String(const String& str); //文字列をコピーしてコンストラクト
	String(int val);//コンストラクタ
	String(float  val);//コンストラクタ	
	String(double val);//コンストラクタ
	String(const _TCHAR* format, ...);//コンストラクタ
	String(_TCHAR c);//コンストラクタ
#ifdef _UNICODE
	String(const std::wstring&);//コンストラクタ
#else
	//String(const std::string&);//コンストラクタ
#endif
	
	
	~String();//デストラクタ


	//文字列のクリア
	void clear();

	//内部バッファにアクセス
	_TCHAR* getbuf() const;
	
	//文字領域の強制確保
	void reserve(int size);
	
	//文字列の長さを得る
	int len() const;
	
	const _TCHAR& at(int i) const;//i番目の文字を得る
	_TCHAR& at(int i) {return m_buf[i];}//i番目の文字を得る
	
	//i番目の文字にcを代入
	void set(int i, _TCHAR c);
	
	//formatに従い文字を代入
	String& format(const _TCHAR* format, ...);
	
	//formatに従い文字を追加
	String& add_format(const _TCHAR* format, ...);
	
	//文字の検索
	//見つからなかった場合は-1を返す
	//見つかった場合はその場所を返す
	//offsetは検索開始位置
	int  find(const _TCHAR c, int offset=0) const;

	//文字列の検索
	//見つからなかった場合は-1を返す
	//見つかった場合はその場所を返す
	//offsetは検索開始位置
	int  find(const _TCHAR* str, int offset=0) const;

	//一文字置換
	//all=trueの場合は、全体を置換 falseの場合は先頭のみを置換
	//offsetは検索開始位置
	String replace(const _TCHAR from, const _TCHAR to, int all=1, int offset=0);

	//部分文字列置換
	//all=trueの場合は、全体を置換 falseの場合は先頭のみを置換
	//offsetは検索開始位置
	String replace(const _TCHAR* from, const _TCHAR* to, int all=1, int offset=0);
	
	//cと同じ文字をすべて削除
	String delchar(const _TCHAR* c, int offset=0);

	//文字列の左側でwhiteに登録されている文字をすべて削除
	String delleftwhite(const _TCHAR* white);

	//中間文字列を得る
	String mid(int from, int len) const;
	//左中間文字列を得る
	String left(int len) const;
	//右中間文字列を得る
	String right(int len) const;
	//サブストリングの抽出
	String sub(int from, int len) const;
	//すべてを大文字に統一
	String upper() const;
	//すべてを小文字に統一
	String lower() const;
	
	//デリミタを用いて単語の切り出しを行う
	//切り出した単語は元の文字列から削除される
	String getword(const _TCHAR* delm);
	
	//int型に変換可能か？
	int isint()   const;

	//float型に変換可能か？
	int isfloat() const;

	//const char*型に変換
	operator const _TCHAR*() const;

	//int型に変換
	int    atoi() const;
	//float型に変換
	double atof() const;

	String& operator=(const _TCHAR* a);
	String& operator=(const String& a);
#ifdef _UNICODE
	String& operator=(const std::wstring&);
#else
	//String& operator=(const std::string&);
#endif
	
	String& operator=(int   a);
	String& operator=(float a);
	String& operator=(double a);
	String& operator+=(const _TCHAR* a);
	String& operator+=(const _TCHAR a);
	String& operator+=(int   a);
	String& operator+=(float a);
	String& operator+=(double a);
#ifdef _UNICODE
	String& operator+=(const std::wstring& str);
#else
	//String& operator+=(const std::string& str);
#endif
	

	friend   int operator==(const String& a, const String& b);
	friend   int operator==(const String& a, const _TCHAR* b);
	friend   int operator!=(const String& a, const String& b);
	friend   int operator!=(const String& a, const _TCHAR* b);
	friend   String operator+(const String& a, const String& b);
	friend   String operator+(const String& a, const _TCHAR* b);
	friend   String operator+(const _TCHAR* b, const String& a);
	friend   String operator+(const String& a, const _TCHAR b);
	friend   String operator+(const _TCHAR b, String& a);

	friend   String operator+(const String& a, const int& b);
	friend   String operator+(const String& a, const float& b);
	friend   String operator+(const String& a, const double& b);
	friend   bool   operator>(const String& a, const String& b);
	friend   bool   operator<(const String& a, const String& b);

	bool equals(const String& a);
//アトリビュート
private:
	_TCHAR* m_buf;
	int   m_allocsize;
};
#if defined(__BASE_STREAM_H_INCLUDED__)
/* キャストoperator があるので定義しなくて良い
BaseOStream& operator<<(BaseOStream& ostr, const String& str) 
{
	ostr << (const char*)str;
	return ostr;
}
*/
inline
BaseIStream& operator>>(BaseIStream& istr, String& str) 
{
	if (istr.ascii()) {
		_TCHAR tmp[1024];
		istr.get_token(tmp, 1024);
		str = tmp;
	} else {
		DWORD len;
		istr >> len;
		std::vector<_TCHAR> tmp((len+1)*sizeof(_TCHAR), '\0');
		istr.read(&(*tmp.begin()), len);

		str = (_TCHAR*)(&(*tmp.begin()));	// modified by XXX 2004.7.15
	}
	return istr;
}

inline
BaseOStream& operator<<(BaseOStream& ostr, const String& str) 
{
	if (ostr.ascii()) {
		ostr << (const _TCHAR*)str;
	} else {
		DWORD len = str.len();
		ostr << len;
		ostr.write(str.getbuf(), len);
	}
	return ostr;
}

#endif


}

#endif

