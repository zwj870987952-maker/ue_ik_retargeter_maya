// Minimal standalone containers and double precision math for UE 5.8 PBIK.
// Deliberately follows Unreal's quaternion / rotator conventions, not Maya's.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

using int32 = int32_t;
using uint8 = uint8_t;
using FName = int32;
constexpr int32 INDEX_NONE = -1;
constexpr double SMALL_NUMBER = 1.e-8;
constexpr double KINDA_SMALL_NUMBER = 1.e-4;
constexpr double PI = 3.14159265358979323846;
#define PBIK_API
#define UENUM(...)
#define USTRUCT(...)
#define UPROPERTY(...)
#define GENERATED_BODY(...)
#define DECLARE_CYCLE_STAT(...)
#define SCOPE_CYCLE_COUNTER(...)
#define LLM_SCOPE_BYNAME(...)
#define TEXT(x) x
#define UE_LOGF(...)
#define check(x) do { if (!(x)) throw std::runtime_error("PBIK invariant: " #x); } while(0)
#define ensureMsgf(x,...) (x)

template<int N> struct TInlineAllocator {};
template<class T, class Alloc=void> class TArray : public std::vector<T> {
public:
    using std::vector<T>::vector;
    int32 Num() const { return static_cast<int32>(this->size()); }
    bool IsEmpty() const { return this->empty(); }
    bool IsValidIndex(int32 i) const { return i>=0 && i<Num(); }
    void Reset() { this->clear(); }
    void Empty() { this->clear(); }
    void Reserve(int32 n) { this->reserve(n); }
    int32 Add(const T& item) { this->push_back(item); return Num()-1; }
    template<class... A> int32 Emplace(A&&... a) { this->emplace_back(std::forward<A>(a)...); return Num()-1; }
    void Sort() { std::sort(this->begin(),this->end()); }
    template<class Predicate> int32 IndexOfByPredicate(Predicate p) const {
        for (int32 i=0;i<Num();++i) if(p((*this)[i])) return i;
        return INDEX_NONE;
    }
};
template<class T> struct TArrayView {
    T* data; int32 count;
    template<class A> TArrayView(const TArray<T,A>& a):data(const_cast<T*>(a.data())),count(a.Num()){}
    TArrayView(T* p,int32 n):data(p),count(n){}
    int32 Num() const {return count;}
    T& operator[](int32 i) const {return data[i];}
};
namespace Algo { template<class T> void Reverse(T& a) {std::reverse(a.begin(),a.end());} }
template<class T> class TSharedPtr : public std::shared_ptr<T> {
public:
    using std::shared_ptr<T>::shared_ptr;
    TSharedPtr(const std::shared_ptr<T>& p):std::shared_ptr<T>(p){}
    template<class U> TSharedPtr(const TSharedPtr<U>& p):std::shared_ptr<T>(p){}
    T* Get() const {return this->get();}
};
template<class T,class... A> TSharedPtr<T> MakeShared(A&&... a) {return std::make_shared<T>(std::forward<A>(a)...);}

struct FVector {
    double X=0,Y=0,Z=0;
    FVector()=default;
    FVector(double x,double y,double z):X(x),Y(y),Z(z){}
    double& operator[](int i) {return i==0?X:i==1?Y:Z;}
    double operator[](int i) const {return i==0?X:i==1?Y:Z;}
    FVector operator+(const FVector& b) const {return {X+b.X,Y+b.Y,Z+b.Z};}
    FVector operator-(const FVector& b) const {return {X-b.X,Y-b.Y,Z-b.Z};}
    FVector operator-() const {return {-X,-Y,-Z};}
    FVector operator*(double s) const {return {X*s,Y*s,Z*s};}
    FVector operator*(const FVector& b) const {return {X*b.X,Y*b.Y,Z*b.Z};}
    FVector operator/(double s) const {return *this*(1/s);}
    FVector& operator+=(const FVector& b) {X+=b.X;Y+=b.Y;Z+=b.Z;return *this;}
    FVector& operator-=(const FVector& b) {X-=b.X;Y-=b.Y;Z-=b.Z;return *this;}
    FVector& operator*=(double s) {X*=s;Y*=s;Z*=s;return *this;}
    FVector& operator*=(const FVector& b) {X*=b.X;Y*=b.Y;Z*=b.Z;return *this;}
    double SizeSquared() const {return X*X+Y*Y+Z*Z;}
    double Size() const {return std::sqrt(SizeSquared());}
    FVector GetSafeNormal(double tolerance=SMALL_NUMBER) const {double s=SizeSquared();return s>tolerance?*this/std::sqrt(s):FVector();}
    FVector GetClampedToMaxSize(double max) const {double s=Size();return s>max?*this*(max/s):*this;}
    void ToDirectionAndLength(FVector& dir,float& size) const {size=static_cast<float>(Size());dir=size>SMALL_NUMBER?*this/size:FVector();}
    double Dot(const FVector& b) const {return X*b.X+Y*b.Y+Z*b.Z;}
    FVector Cross(const FVector& b) const {return {Y*b.Z-Z*b.Y,Z*b.X-X*b.Z,X*b.Y-Y*b.X};}
    static double DotProduct(const FVector& a,const FVector& b) {return a.Dot(b);}
    static FVector CrossProduct(const FVector& a,const FVector& b) {return a.Cross(b);}
    static FVector VectorPlaneProject(const FVector& v,const FVector& n) {return v-n*v.Dot(n);}
    static const FVector ZeroVector;
};
inline const FVector FVector::ZeroVector{};
inline FVector operator*(double s,const FVector& v) {return v*s;}

struct FQuat;
struct FRotator {
    double Pitch=0,Yaw=0,Roll=0;
    FRotator()=default;
    FRotator(double p,double y,double r):Pitch(p),Yaw(y),Roll(r){}
    FRotator operator*(double s) const {return {Pitch*s,Yaw*s,Roll*s};}
    FQuat Quaternion() const;
    static double NormalizeAxis(double a) {a=std::fmod(a,360.);if(a<0)a+=360.;if(a>180.)a-=360.;return a;}
    static const FRotator ZeroRotator;
};
inline const FRotator FRotator::ZeroRotator{};
struct FQuat {
    double X=0,Y=0,Z=0,W=1;
    FQuat()=default;
    FQuat(double x,double y,double z,double w):X(x),Y(y),Z(z),W(w){}
    FQuat(FVector axis,double angle) {double s=std::sin(angle*.5);X=axis.X*s;Y=axis.Y*s;Z=axis.Z*s;W=std::cos(angle*.5);}
    FQuat operator*(const FQuat& b) const {
        return {W*b.X+X*b.W+Y*b.Z-Z*b.Y,W*b.Y-X*b.Z+Y*b.W+Z*b.X,
                W*b.Z+X*b.Y-Y*b.X+Z*b.W,W*b.W-X*b.X-Y*b.Y-Z*b.Z};
    }
    FQuat operator*(double s) const {return {X*s,Y*s,Z*s,W*s};}
    FQuat operator+(const FQuat& b) const {return {X+b.X,Y+b.Y,Z+b.Z,W+b.W};}
    FQuat& operator+=(const FQuat& b) {X+=b.X;Y+=b.Y;Z+=b.Z;W+=b.W;return *this;}
    FQuat& operator*=(double s) {X*=s;Y*=s;Z*=s;W*=s;return *this;}
    double Dot(const FQuat& b) const {return X*b.X+Y*b.Y+Z*b.Z+W*b.W;}
    double Size() const {return std::sqrt(Dot(*this));}
    void Normalize() {double s=Dot(*this);if(s>=SMALL_NUMBER)*this*=1./std::sqrt(s);else *this=Identity;}
    FQuat GetNormalized() const {auto q=*this;q.Normalize();return q;}
    FQuat Inverse() const {return {-X,-Y,-Z,W};}
    FVector RotateVector(const FVector& v) const {FVector q(X,Y,Z);FVector t=2.*q.Cross(v);return v+W*t+q.Cross(t);}
    FVector operator*(const FVector& v) const {return RotateVector(v);}
    static FQuat FastLerp(const FQuat& a,const FQuat& b,double t) {double bias=a.Dot(b)>=0?1.:-1.;return a*(bias*(1.-t))+b*t;}
    static FQuat Slerp(const FQuat& a,const FQuat& b,double t) {
        double dot=a.Dot(b),sign=dot>=0?1.:-1.;dot=std::abs(dot);
        if(dot>=.9999)return FastLerp(a,b,t).GetNormalized();
        double angle=std::acos(std::min(1.,dot)),sn=std::sin(angle);
        return (a*(std::sin((1-t)*angle)/sn)+b*(sign*std::sin(t*angle)/sn)).GetNormalized();
    }
    static FQuat FindBetweenVectors(const FVector& a,const FVector& b) {
        double norm=std::sqrt(a.SizeSquared()*b.SizeSquared()),w=norm+a.Dot(b);
        FQuat q;
        if(w>=1.e-6*norm){FVector c=a.Cross(b);q={c.X,c.Y,c.Z,w};}
        else if(std::abs(a.X)>std::abs(a.Y))q={-a.Z,0,a.X,0};
        else q={0,-a.Z,a.Y,0};
        return q.GetNormalized();
    }
    static FQuat FindBetweenNormals(const FVector& a,const FVector& b) {return FindBetweenVectors(a,b);}
    FRotator Rotator() const {
        double singular=Z*X-W*Y,yawY=2*(W*Z+X*Y),yawX=1-2*(Y*Y+Z*Z),r=180./PI;
        if(singular<-.4999995)return {-90,FRotator::NormalizeAxis(-2*std::atan2(X,W)*r),0};
        if(singular>.4999995)return {90,FRotator::NormalizeAxis(2*std::atan2(X,W)*r),0};
        return {std::asin(std::clamp(2*singular,-1.,1.))*r,std::atan2(yawY,yawX)*r,
                std::atan2(-2*(W*X+Y*Z),1-2*(X*X+Y*Y))*r};
    }
    FVector Euler() const {auto r=Rotator();return {r.Roll,r.Pitch,r.Yaw};}
    static FQuat MakeFromEuler(const FVector& e) {return FRotator(e.Y,e.Z,e.X).Quaternion();}
    static const FQuat Identity;
};
inline const FQuat FQuat::Identity{};
inline FQuat FRotator::Quaternion() const {
    double p=std::fmod(Pitch,360.)*PI/360.,y=std::fmod(Yaw,360.)*PI/360.,r=std::fmod(Roll,360.)*PI/360.;
    double sp=std::sin(p),cp=std::cos(p),sy=std::sin(y),cy=std::cos(y),sr=std::sin(r),cr=std::cos(r);
    return {cr*sp*sy-sr*cp*cy,-cr*sp*cy-sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy};
}
struct FTransform {
    FVector p,s{1,1,1};FQuat q;
    FVector GetLocation() const {return p;} FQuat GetRotation() const {return q;} FVector GetScale3D() const {return s;}
    void SetLocation(const FVector& v){p=v;} void SetRotation(const FQuat& v){q=v;} void SetScale3D(const FVector& v){s=v;}
};
struct FMatrix {double M[4][4]{};};
struct FRotationMatrix {
    static FMatrix Make(const FQuat& q) {
        FMatrix m;FVector basis[3]={{1,0,0},{0,1,0},{0,0,1}};
        for(int i=0;i<3;++i){auto v=q*basis[i];for(int j=0;j<3;++j)m.M[i][j]=v[j];}m.M[3][3]=1;return m;
    }
};
struct FMath {
    template<class T,class U> static auto Max(T a,U b){return a>b?a:b;}
    template<class T,class U> static auto Min(T a,U b){return a<b?a:b;}
    template<class T,class U,class V> static auto Clamp(T v,U a,V b){return v<a?a:v>b?b:v;}
    template<class T> static T Abs(T a){return std::abs(a);}
    template<class T> static auto Sqrt(T a){return std::sqrt(a);}
    template<class T,class U> static auto Pow(T a,U b){return std::pow(a,b);}
    template<class T> static auto Acos(T a){return std::acos(std::clamp(double(a),-1.,1.));}
    template<class T> static double DegreesToRadians(T a){return a*PI/180.;}
    template<class T> static double RadiansToDegrees(T a){return a*180./PI;}
    template<class T,class U> static auto Lerp(const T& a,const T& b,U t){return a+(b-a)*t;}
    template<class T,class U> static bool IsNearlyEqual(T a,U b,double tol=KINDA_SMALL_NUMBER){return std::abs(a-b)<=tol;}
    template<class T> static bool IsNearlyZero(T a,double tol=KINDA_SMALL_NUMBER){return std::abs(a)<=tol;}
};
