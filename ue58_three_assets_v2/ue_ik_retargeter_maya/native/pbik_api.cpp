#include "pbik_compat.h"
#include "Core/PBIKBody.h"
#include "Core/PBIKConstraint.h"
#include "Core/PBIKSolver.h"
#include <string>

#define EXPORT extern "C" __declspec(dllexport)
struct BoneSettings {
    double rotation_stiffness,position_stiffness;
    int32 limits[3];
    double minimum[3],maximum[3],preferred[3];
    int32 use_preferred;
};
struct SolverSettings {
    int32 iterations,sub_iterations;
    double mass;
    int32 root_behavior,allow_stretch;
    double pull_chain,max_angle,over_relaxation;
    double pre_pull[8];
};
struct Goal {
    int32 bone;
    double position[3],rotation[4];
    double position_alpha,rotation_alpha,strength,pull_chain,pin_rotation;
    int32 chain_depth;
};
struct Handle {FPBIKSolver solver;int32 bone_count,goal_count;};
static thread_local std::string error;
static FVector vector(const double* p) {return {p[0],p[1],p[2]};}
static FQuat quat(const double* q) {return FQuat(q[0],q[1],q[2],q[3]).GetNormalized();}
EXPORT const char* pbik_error() {return error.c_str();}
EXPORT int32 pbik_abi() {return 1;}
EXPORT void* pbik_create(const int32* parents,int32 count,int32 root,const double* reference,const int32* goals,int32 goal_count) {
    try {
        error.clear();check(count>0 && count<=4096 && root>=0 && root<count && goal_count>0 && goal_count<=count);
        auto h=std::make_unique<Handle>();h->bone_count=count;h->goal_count=goal_count;
        for(int32 i=0;i<count;++i){check(parents[i]>=-1 && parents[i]<i);h->solver.AddBone(i,parents[i],vector(reference+i*10),quat(reference+i*10+3),i==root);}
        for(int32 i=0;i<goal_count;++i){check(goals[i]>=0 && goals[i]<count);check(h->solver.AddEffector(goals[i])>=0);}
        check(h->solver.Initialize());return h.release();
    } catch(const std::exception& e){error=e.what();return nullptr;}
}
EXPORT void pbik_destroy(void* handle) {delete static_cast<Handle*>(handle);}
EXPORT int32 pbik_solve(void* handle,const double* input,const BoneSettings* bones,const Goal* goals,const SolverSettings* settings,double* output) {
    try {
        error.clear();auto h=static_cast<Handle*>(handle);check(h);
        for(int32 i=0;i<h->bone_count;++i){
            for(int j=0;j<10;++j)check(std::isfinite(input[i*10+j]));
            FTransform t;t.SetLocation(vector(input+i*10));t.SetRotation(quat(input+i*10+3));t.SetScale3D(vector(input+i*10+7));h->solver.SetBoneTransform(i,t);
            if(auto s=h->solver.GetBoneSettings(i)){
                *s=PBIK::FBoneSettings();const auto& b=bones[i];
                s->RotationStiffness=float(b.rotation_stiffness);s->PositionStiffness=float(b.position_stiffness);
                s->X=PBIK::ELimitType(b.limits[0]);s->Y=PBIK::ELimitType(b.limits[1]);s->Z=PBIK::ELimitType(b.limits[2]);
                s->MinX=float(b.minimum[0]);s->MinY=float(b.minimum[1]);s->MinZ=float(b.minimum[2]);
                s->MaxX=float(b.maximum[0]);s->MaxY=float(b.maximum[1]);s->MaxZ=float(b.maximum[2]);
                s->bUsePreferredAngles=b.use_preferred!=0;s->PreferredAngles=FRotator(b.preferred[1],b.preferred[2],b.preferred[0]);
            }
        }
        for(int32 i=0;i<h->goal_count;++i){const auto& g=goals[i];PBIK::FEffectorSettings s;
            s.PositionAlpha=float(g.position_alpha);s.RotationAlpha=float(g.rotation_alpha);s.StrengthAlpha=float(g.strength);
            s.PullChainAlpha=float(g.pull_chain);s.PinRotation=float(g.pin_rotation);s.ChainDepth=g.chain_depth;
            h->solver.SetEffectorGoal(i,vector(g.position),quat(g.rotation),s);
        }
        FPBIKSolverSettings s;s.Iterations=settings->iterations;s.SubIterations=settings->sub_iterations;s.MassMultiplier=float(settings->mass);
        s.RootBehavior=EPBIKRootBehavior(settings->root_behavior);s.bAllowStretch=settings->allow_stretch!=0;
        s.GlobalPullChainAlpha=float(settings->pull_chain);s.MaxAngle=float(settings->max_angle);s.OverRelaxation=float(settings->over_relaxation);
        auto& p=s.PrePullRootSettings;const double* v=settings->pre_pull;
        p.RotationAlpha=float(v[0]);p.RotationAlphaX=float(v[1]);p.RotationAlphaY=float(v[2]);p.RotationAlphaZ=float(v[3]);
        p.PositionAlpha=float(v[4]);p.PositionAlphaX=float(v[5]);p.PositionAlphaY=float(v[6]);p.PositionAlphaZ=float(v[7]);
        h->solver.Solve(s);
        for(int32 i=0;i<h->bone_count;++i){FTransform t;h->solver.GetBoneGlobalTransform(i,t);auto p=t.GetLocation();auto q=t.GetRotation();auto scale=t.GetScale3D();
            double values[]={p.X,p.Y,p.Z,q.X,q.Y,q.Z,q.W,scale.X,scale.Y,scale.Z};
            for(int j=0;j<10;++j){check(std::isfinite(values[j]));output[i*10+j]=values[j];}
        }
        return 1;
    } catch(const std::exception& e){error=e.what();return 0;}
}
