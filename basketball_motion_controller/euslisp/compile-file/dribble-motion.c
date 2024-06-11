/* ./dribble-motion.c :  entry=dribble_motion */
/* compiled by EusLisp 9.29( 1.2.5) for Linux64 created on ip-10-0-1-151(Tue Oct 25 10:11:33 PST 2022) */
#include "eus.h"
#include "dribble-motion.h"
#pragma init (register_dribble_motion)
extern double fabs();
extern pointer fcallx();
static void init_ftab();
extern pointer loadglobal(),storeglobal();
static pointer module,*qv,codevec,quotevec;
extern pointer ___dribble_motion();
extern pointer build_quote_vector();
static int register_dribble_motion()
  { add_module_initializer("___dribble-motion", ___dribble_motion);}

static pointer dribble_F1__gc_print();
static pointer dribble_F2start_ab();
static pointer dribble_F3stop_ab();
static pointer dribble_F4start_st();
static pointer dribble_F5stop_st();
static pointer dribble_F6start_ee();
static pointer dribble_F7stop_ee();
static pointer dribble_F8start_d_mode();
static pointer dribble_F9stop_d_mode();
static pointer dribble_F10start_d_motion();
static pointer dribble_F11stop_d_motion();
static pointer dribble_F12start_dribble_mode();
static pointer dribble_F13stop_dribble_mode();
static pointer dribble_F14move();
static pointer dribble_F15lower_waist();

/*__gc_print*/
static pointer dribble_F1__gc_print(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=2) maerror();
	local[0]= fqv[0];
	ctx->vsp=local+1;
	w=(pointer)PRINT(ctx,1,local+0); /*print*/
	local[0]= w;
dribble_BLK16:
	ctx->vsp=local; return(local[0]);}

/*do-until-key-w/o-gc*/
static pointer dribble_F17(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<0) maerror();
dribble_RST19:
	ctx->vsp=local+0;
	local[0]= minilist(ctx,&argv[n],n-0);
	local[1]= fqv[1];
	local[2]= fqv[2];
	local[3]= fqv[3];
	local[4]= fqv[4];
	ctx->vsp=local+5;
	w=(pointer)LIST(ctx,1,local+4); /*list*/
	ctx->vsp=local+4;
	local[3]= cons(ctx,local[3],w);
	ctx->vsp=local+4;
	w=(pointer)LIST(ctx,1,local+3); /*list*/
	ctx->vsp=local+3;
	local[2]= cons(ctx,local[2],w);
	ctx->vsp=local+3;
	w=(pointer)LIST(ctx,1,local+2); /*list*/
	local[2]= w;
	local[3]= fqv[5];
	local[4]= fqv[6];
	local[5]= fqv[7];
	local[6]= fqv[2];
	local[7]= fqv[8];
	ctx->vsp=local+8;
	w=(pointer)LIST(ctx,1,local+7); /*list*/
	ctx->vsp=local+7;
	w = cons(ctx,local[6],w);
	ctx->vsp=local+6;
	local[5]= cons(ctx,local[5],w);
	ctx->vsp=local+6;
	w=(pointer)LIST(ctx,1,local+5); /*list*/
	ctx->vsp=local+5;
	local[4]= cons(ctx,local[4],w);
	local[5]= local[0];
	local[6]= NIL;
	ctx->vsp=local+7;
	w=(pointer)APPEND(ctx,2,local+5); /*append*/
	ctx->vsp=local+5;
	w = cons(ctx,local[4],w);
	ctx->vsp=local+4;
	local[3]= cons(ctx,local[3],w);
	local[4]= fqv[9];
	local[5]= fqv[10];
	local[6]= fqv[11];
	local[7]= fqv[7];
	local[8]= fqv[2];
	local[9]= fqv[12];
	ctx->vsp=local+10;
	w=(pointer)LIST(ctx,1,local+9); /*list*/
	ctx->vsp=local+9;
	w = cons(ctx,local[8],w);
	ctx->vsp=local+8;
	local[7]= cons(ctx,local[7],w);
	ctx->vsp=local+8;
	w=(pointer)LIST(ctx,1,local+7); /*list*/
	ctx->vsp=local+7;
	local[6]= cons(ctx,local[6],w);
	ctx->vsp=local+7;
	w=(pointer)LIST(ctx,1,local+6); /*list*/
	ctx->vsp=local+6;
	local[5]= cons(ctx,local[5],w);
	ctx->vsp=local+6;
	w=(pointer)LIST(ctx,1,local+5); /*list*/
	local[5]= w;
	local[6]= fqv[13];
	local[7]= fqv[10];
	local[8]= fqv[14];
	local[9]= fqv[10];
	local[10]= fqv[15];
	local[11]= fqv[15];
	ctx->vsp=local+12;
	w=(pointer)LIST(ctx,1,local+11); /*list*/
	ctx->vsp=local+11;
	w = cons(ctx,local[10],w);
	ctx->vsp=local+10;
	w = cons(ctx,local[9],w);
	ctx->vsp=local+9;
	local[8]= cons(ctx,local[8],w);
	ctx->vsp=local+9;
	w=(pointer)LIST(ctx,1,local+8); /*list*/
	ctx->vsp=local+8;
	w = cons(ctx,local[7],w);
	ctx->vsp=local+7;
	local[6]= cons(ctx,local[6],w);
	ctx->vsp=local+7;
	w=(pointer)LIST(ctx,1,local+6); /*list*/
	ctx->vsp=local+6;
	w = cons(ctx,local[5],w);
	ctx->vsp=local+5;
	local[4]= cons(ctx,local[4],w);
	ctx->vsp=local+5;
	w=(pointer)LIST(ctx,1,local+4); /*list*/
	ctx->vsp=local+4;
	w = cons(ctx,local[3],w);
	ctx->vsp=local+3;
	w = cons(ctx,local[2],w);
	ctx->vsp=local+2;
	w = cons(ctx,local[1],w);
	local[0]= w;
dribble_BLK18:
	ctx->vsp=local; return(local[0]);}

/*:init*/
static pointer dribble_M20handtrajectorycontroller_init(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=2) maerror();
	argv[0]->c.obj.iv[1] = loadglobal(fqv[16]);
	argv[0]->c.obj.iv[2] = loadglobal(fqv[17]);
	argv[0]->c.obj.iv[3] = loadglobal(fqv[18]);
	argv[0]->c.obj.iv[4] = NIL;
	argv[0]->c.obj.iv[5] = NIL;
	argv[0]->c.obj.iv[6] = makeint((eusinteger_t)0L);
	argv[0]->c.obj.iv[7] = NIL;
	argv[0]->c.obj.iv[8] = makeflt(2.5000000000000000000000e-01);
	argv[0]->c.obj.iv[9] = makeflt(7.9999999999999982236432e-01);
	argv[0]->c.obj.iv[10] = makeflt(1.0000000000000000000000e+00);
	argv[0]->c.obj.iv[11] = makeflt(2.0000000000000000416334e-03);
	storeglobal(fqv[19],makeflt(0.0000000000000000000000e+00));
	local[0]= makeflt(0.0000000000000000000000e+00);
	local[1]= makeflt(0.0000000000000000000000e+00);
	local[2]= makeflt(0.0000000000000000000000e+00);
	ctx->vsp=local+3;
	w=(pointer)MKFLTVEC(ctx,3,local+0); /*float-vector*/
	argv[0]->c.obj.iv[12] = w;
	local[0]= makeflt(0.0000000000000000000000e+00);
	local[1]= makeflt(0.0000000000000000000000e+00);
	local[2]= makeflt(0.0000000000000000000000e+00);
	ctx->vsp=local+3;
	w=(pointer)MKFLTVEC(ctx,3,local+0); /*float-vector*/
	argv[0]->c.obj.iv[13] = w;
	ctx->vsp=local+0;
	w=(*ftab[0])(ctx,0,local+0,&ftab[0],fqv[20]); /*make-coords*/
	argv[0]->c.obj.iv[14] = w;
	argv[0]->c.obj.iv[15] = NIL;
	argv[0]->c.obj.iv[16] = NIL;
	argv[0]->c.obj.iv[17] = NIL;
	argv[0]->c.obj.iv[18] = NIL;
	argv[0]->c.obj.iv[19] = NIL;
	argv[0]->c.obj.iv[20] = NIL;
	local[0]= loadglobal(fqv[21]);
	local[1]= fqv[22];
	local[2]= fqv[23];
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,3,local+0); /*send*/
	local[0]= w;
	local[1]= makeflt(9.9999999999999977795540e-02);
	ctx->vsp=local+2;
	w=(pointer)LIST(ctx,2,local+0); /*list*/
	local[0]= w;
	local[1]= loadglobal(fqv[21]);
	local[2]= fqv[24];
	local[3]= fqv[23];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,3,local+1); /*send*/
	local[1]= w;
	local[2]= makeflt(9.9999999999999950039964e-03);
	ctx->vsp=local+3;
	w=(pointer)LIST(ctx,2,local+1); /*list*/
	local[1]= w;
	local[2]= loadglobal(fqv[21]);
	local[3]= fqv[25];
	local[4]= fqv[23];
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,3,local+2); /*send*/
	local[2]= w;
	local[3]= makeflt(9.9999999999999977795540e-02);
	ctx->vsp=local+4;
	w=(pointer)LIST(ctx,2,local+2); /*list*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)LIST(ctx,3,local+0); /*list*/
	argv[0]->c.obj.iv[21] = w;
	w = argv[0];
	local[0]= w;
dribble_BLK21:
	ctx->vsp=local; return(local[0]);}

/*:object-state*/
static pointer dribble_M22handtrajectorycontroller_object_state(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT25;}
	local[0]= NIL;
dribble_ENT25:
dribble_ENT24:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF26;
	argv[0]->c.obj.iv[1] = local[0];
	local[1]= argv[0]->c.obj.iv[1];
	goto dribble_IF27;
dribble_IF26:
	local[1]= NIL;
dribble_IF27:
	w = argv[0]->c.obj.iv[1];
	local[0]= w;
dribble_BLK23:
	ctx->vsp=local; return(local[0]);}

/*:tactile-state*/
static pointer dribble_M28handtrajectorycontroller_tactile_state(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT31;}
	local[0]= NIL;
dribble_ENT31:
dribble_ENT30:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF32;
	argv[0]->c.obj.iv[2] = local[0];
	local[1]= argv[0]->c.obj.iv[2];
	goto dribble_IF33;
dribble_IF32:
	local[1]= NIL;
dribble_IF33:
	w = argv[0]->c.obj.iv[2];
	local[0]= w;
dribble_BLK29:
	ctx->vsp=local; return(local[0]);}

/*:joint-states*/
static pointer dribble_M34handtrajectorycontroller_joint_states(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT37;}
	local[0]= NIL;
dribble_ENT37:
dribble_ENT36:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF38;
	argv[0]->c.obj.iv[3] = local[0];
	local[1]= argv[0]->c.obj.iv[3];
	goto dribble_IF39;
dribble_IF38:
	local[1]= NIL;
dribble_IF39:
	w = argv[0]->c.obj.iv[3];
	local[0]= w;
dribble_BLK35:
	ctx->vsp=local; return(local[0]);}

/*:ref-pose*/
static pointer dribble_M40handtrajectorycontroller_ref_pose(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT43;}
	local[0]= NIL;
dribble_ENT43:
dribble_ENT42:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF44;
	argv[0]->c.obj.iv[4] = local[0];
	local[1]= argv[0]->c.obj.iv[4];
	goto dribble_IF45;
dribble_IF44:
	local[1]= NIL;
dribble_IF45:
	w = argv[0]->c.obj.iv[4];
	local[0]= w;
dribble_BLK41:
	ctx->vsp=local; return(local[0]);}

/*:target-z-start*/
static pointer dribble_M46handtrajectorycontroller_target_z_start(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT49;}
	local[0]= NIL;
dribble_ENT49:
dribble_ENT48:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF50;
	argv[0]->c.obj.iv[5] = local[0];
	local[1]= argv[0]->c.obj.iv[5];
	goto dribble_IF51;
dribble_IF50:
	local[1]= NIL;
dribble_IF51:
	w = argv[0]->c.obj.iv[5];
	local[0]= w;
dribble_BLK47:
	ctx->vsp=local; return(local[0]);}

/*:target-z-goal*/
static pointer dribble_M52handtrajectorycontroller_target_z_goal(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT55;}
	local[0]= NIL;
dribble_ENT55:
dribble_ENT54:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF56;
	argv[0]->c.obj.iv[6] = local[0];
	local[1]= argv[0]->c.obj.iv[6];
	goto dribble_IF57;
dribble_IF56:
	local[1]= NIL;
dribble_IF57:
	w = argv[0]->c.obj.iv[6];
	local[0]= w;
dribble_BLK53:
	ctx->vsp=local; return(local[0]);}

/*:target-pitch-start*/
static pointer dribble_M58handtrajectorycontroller_target_pitch_start(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT61;}
	local[0]= NIL;
dribble_ENT61:
dribble_ENT60:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF62;
	argv[0]->c.obj.iv[7] = local[0];
	local[1]= argv[0]->c.obj.iv[7];
	goto dribble_IF63;
dribble_IF62:
	local[1]= NIL;
dribble_IF63:
	w = argv[0]->c.obj.iv[7];
	local[0]= w;
dribble_BLK59:
	ctx->vsp=local; return(local[0]);}

/*:target-pitch-goal*/
static pointer dribble_M64handtrajectorycontroller_target_pitch_goal(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT67;}
	local[0]= NIL;
dribble_ENT67:
dribble_ENT66:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF68;
	argv[0]->c.obj.iv[8] = local[0];
	local[1]= argv[0]->c.obj.iv[8];
	goto dribble_IF69;
dribble_IF68:
	local[1]= NIL;
dribble_IF69:
	w = argv[0]->c.obj.iv[8];
	local[0]= w;
dribble_BLK65:
	ctx->vsp=local; return(local[0]);}

/*:motion-period*/
static pointer dribble_M70handtrajectorycontroller_motion_period(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT73;}
	local[0]= NIL;
dribble_ENT73:
dribble_ENT72:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF74;
	argv[0]->c.obj.iv[9] = local[0];
	local[1]= argv[0]->c.obj.iv[9];
	goto dribble_IF75;
dribble_IF74:
	local[1]= NIL;
dribble_IF75:
	w = argv[0]->c.obj.iv[9];
	local[0]= w;
dribble_BLK71:
	ctx->vsp=local; return(local[0]);}

/*:scale-period*/
static pointer dribble_M76handtrajectorycontroller_scale_period(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT79;}
	local[0]= NIL;
dribble_ENT79:
dribble_ENT78:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF80;
	argv[0]->c.obj.iv[10] = local[0];
	local[1]= argv[0]->c.obj.iv[10];
	goto dribble_IF81;
dribble_IF80:
	local[1]= NIL;
dribble_IF81:
	w = argv[0]->c.obj.iv[10];
	local[0]= w;
dribble_BLK77:
	ctx->vsp=local; return(local[0]);}

/*:dt*/
static pointer dribble_M82handtrajectorycontroller_dt(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT85;}
	local[0]= NIL;
dribble_ENT85:
dribble_ENT84:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF86;
	argv[0]->c.obj.iv[11] = local[0];
	local[1]= argv[0]->c.obj.iv[11];
	goto dribble_IF87;
dribble_IF86:
	local[1]= NIL;
dribble_IF87:
	w = argv[0]->c.obj.iv[11];
	local[0]= w;
dribble_BLK83:
	ctx->vsp=local; return(local[0]);}

/*:time*/
static pointer dribble_M88handtrajectorycontroller_time(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT91;}
	local[0]= NIL;
dribble_ENT91:
dribble_ENT90:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF92;
	local[1]= local[0];
	storeglobal(fqv[19],local[1]);
	goto dribble_IF93;
dribble_IF92:
	local[1]= NIL;
dribble_IF93:
	w = loadglobal(fqv[19]);
	local[0]= w;
dribble_BLK89:
	ctx->vsp=local; return(local[0]);}

/*:dt-list*/
static pointer dribble_M94handtrajectorycontroller_dt_list(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT97;}
	local[0]= NIL;
dribble_ENT97:
dribble_ENT96:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF98;
	argv[0]->c.obj.iv[16] = local[0];
	local[1]= argv[0]->c.obj.iv[16];
	goto dribble_IF99;
dribble_IF98:
	local[1]= NIL;
dribble_IF99:
	w = argv[0]->c.obj.iv[16];
	local[0]= w;
dribble_BLK95:
	ctx->vsp=local; return(local[0]);}

/*:target-hand-pos*/
static pointer dribble_M100handtrajectorycontroller_target_hand_pos(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT103;}
	local[0]= NIL;
dribble_ENT103:
dribble_ENT102:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF104;
	argv[0]->c.obj.iv[12] = local[0];
	local[1]= argv[0]->c.obj.iv[12];
	goto dribble_IF105;
dribble_IF104:
	local[1]= NIL;
dribble_IF105:
	w = argv[0]->c.obj.iv[12];
	local[0]= w;
dribble_BLK101:
	ctx->vsp=local; return(local[0]);}

/*:target-hand-rpy*/
static pointer dribble_M106handtrajectorycontroller_target_hand_rpy(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT109;}
	local[0]= NIL;
dribble_ENT109:
dribble_ENT108:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF110;
	argv[0]->c.obj.iv[13] = local[0];
	local[1]= argv[0]->c.obj.iv[13];
	goto dribble_IF111;
dribble_IF110:
	local[1]= NIL;
dribble_IF111:
	w = argv[0]->c.obj.iv[13];
	local[0]= w;
dribble_BLK107:
	ctx->vsp=local; return(local[0]);}

/*:target-hand-coords*/
static pointer dribble_M112handtrajectorycontroller_target_hand_coords(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT115;}
	local[0]= NIL;
dribble_ENT115:
dribble_ENT114:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF116;
	argv[0]->c.obj.iv[14] = local[0];
	local[1]= argv[0]->c.obj.iv[14];
	goto dribble_IF117;
dribble_IF116:
	local[1]= NIL;
dribble_IF117:
	w = argv[0]->c.obj.iv[14];
	local[0]= w;
dribble_BLK113:
	ctx->vsp=local; return(local[0]);}

/*:target-angle-vector*/
static pointer dribble_M118handtrajectorycontroller_target_angle_vector(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT121;}
	local[0]= NIL;
dribble_ENT121:
dribble_ENT120:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF122;
	argv[0]->c.obj.iv[15] = local[0];
	local[1]= argv[0]->c.obj.iv[15];
	goto dribble_IF123;
dribble_IF122:
	local[1]= NIL;
dribble_IF123:
	w = argv[0]->c.obj.iv[15];
	local[0]= w;
dribble_BLK119:
	ctx->vsp=local; return(local[0]);}

/*:target-hand-pos-list*/
static pointer dribble_M124handtrajectorycontroller_target_hand_pos_list(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT127;}
	local[0]= NIL;
dribble_ENT127:
dribble_ENT126:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF128;
	argv[0]->c.obj.iv[17] = local[0];
	local[1]= argv[0]->c.obj.iv[17];
	goto dribble_IF129;
dribble_IF128:
	local[1]= NIL;
dribble_IF129:
	w = argv[0]->c.obj.iv[17];
	local[0]= w;
dribble_BLK125:
	ctx->vsp=local; return(local[0]);}

/*:target-hand-rpy-list*/
static pointer dribble_M130handtrajectorycontroller_target_hand_rpy_list(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT133;}
	local[0]= NIL;
dribble_ENT133:
dribble_ENT132:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF134;
	argv[0]->c.obj.iv[18] = local[0];
	local[1]= argv[0]->c.obj.iv[18];
	goto dribble_IF135;
dribble_IF134:
	local[1]= NIL;
dribble_IF135:
	w = argv[0]->c.obj.iv[18];
	local[0]= w;
dribble_BLK131:
	ctx->vsp=local; return(local[0]);}

/*:target-hand-coords-list*/
static pointer dribble_M136handtrajectorycontroller_target_hand_coords_list(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT139;}
	local[0]= NIL;
dribble_ENT139:
dribble_ENT138:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF140;
	argv[0]->c.obj.iv[19] = local[0];
	local[1]= argv[0]->c.obj.iv[19];
	goto dribble_IF141;
dribble_IF140:
	local[1]= NIL;
dribble_IF141:
	w = argv[0]->c.obj.iv[19];
	local[0]= w;
dribble_BLK137:
	ctx->vsp=local; return(local[0]);}

/*:target-angle-vector-list*/
static pointer dribble_M142handtrajectorycontroller_target_angle_vector_list(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	if (n>=3) { local[0]=(argv[2]); goto dribble_ENT145;}
	local[0]= NIL;
dribble_ENT145:
dribble_ENT144:
	if (n>3) maerror();
	if (local[0]==NIL) goto dribble_IF146;
	argv[0]->c.obj.iv[20] = local[0];
	local[1]= argv[0]->c.obj.iv[20];
	goto dribble_IF147;
dribble_IF146:
	local[1]= NIL;
dribble_IF147:
	w = argv[0]->c.obj.iv[20];
	local[0]= w;
dribble_BLK143:
	ctx->vsp=local; return(local[0]);}

/*:ros-init*/
static pointer dribble_M148handtrajectorycontroller_ros_init(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=2) maerror();
	local[0]= fqv[26];
	ctx->vsp=local+1;
	w=(*ftab[1])(ctx,1,local+0,&ftab[1],fqv[27]); /*ros::roseus*/
	local[0]= fqv[28];
	local[1]= loadglobal(fqv[29]);
	local[2]= makeint((eusinteger_t)1L);
	ctx->vsp=local+3;
	w=(*ftab[2])(ctx,3,local+0,&ftab[2],fqv[30]); /*ros::advertise*/
	local[0]= w;
dribble_BLK149:
	ctx->vsp=local; return(local[0]);}

/*:object-state_cb*/
static pointer dribble_M150handtrajectorycontroller_object_state_cb(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=3) maerror();
	if (argv[2]==NIL) goto dribble_IF152;
	argv[0]->c.obj.iv[1] = argv[2];
	local[0]= argv[0]->c.obj.iv[1];
	goto dribble_IF153;
dribble_IF152:
	local[0]= NIL;
dribble_IF153:
	w = local[0];
	local[0]= w;
dribble_BLK151:
	ctx->vsp=local; return(local[0]);}

/*:dribble-pose*/
static pointer dribble_M154handtrajectorycontroller_dribble_pose(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	ctx->vsp=local+0;
	n=parsekeyparams(fqv[31], &argv[2], n-2, local+0, 0);
	if (n & (1<<0)) goto dribble_KEY156;
	local[0] = NIL;
dribble_KEY156:
	if (local[0]!=NIL) goto dribble_IF157;
	local[1]= loadglobal(fqv[21]);
	local[2]= fqv[32];
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,2,local+1); /*send*/
	local[1]= T;
	local[2]= fqv[33];
	ctx->vsp=local+3;
	w=(pointer)XFORMAT(ctx,2,local+1); /*format*/
	ctx->vsp=local+1;
	w=(pointer)READLINE(ctx,0,local+1); /*read-line*/
	local[1]= fqv[34];
	local[2]= makeint((eusinteger_t)100L);
	local[3]= fqv[35];
	local[4]= T;
	ctx->vsp=local+5;
	w=(pointer)dribble_F15lower_waist(ctx,4,local+1); /*lower-waist*/
	local[1]= T;
	local[2]= fqv[36];
	ctx->vsp=local+3;
	w=(pointer)XFORMAT(ctx,2,local+1); /*format*/
	ctx->vsp=local+1;
	w=(pointer)READLINE(ctx,0,local+1); /*read-line*/
	local[1]= loadglobal(fqv[37]);
	local[2]= fqv[38];
	local[3]= fqv[39];
	local[4]= loadglobal(fqv[21]);
	local[5]= fqv[40];
	local[6]= fqv[41];
	local[7]= fqv[42];
	ctx->vsp=local+8;
	w=(pointer)SEND(ctx,4,local+4); /*send*/
	local[4]= w;
	local[5]= fqv[43];
	local[6]= fqv[40];
	ctx->vsp=local+7;
	w=(*ftab[0])(ctx,4,local+3,&ftab[0],fqv[20]); /*make-coords*/
	local[3]= w;
	local[4]= fqv[39];
	local[5]= loadglobal(fqv[21]);
	local[6]= fqv[40];
	local[7]= fqv[41];
	local[8]= fqv[42];
	ctx->vsp=local+9;
	w=(pointer)SEND(ctx,4,local+5); /*send*/
	local[5]= w;
	local[6]= fqv[44];
	local[7]= makeint((eusinteger_t)600L);
	local[8]= makeint((eusinteger_t)400L);
	local[9]= makeint((eusinteger_t)0L);
	ctx->vsp=local+10;
	w=(pointer)MKFLTVEC(ctx,3,local+7); /*float-vector*/
	local[7]= w;
	ctx->vsp=local+8;
	w=(pointer)SEND(ctx,3,local+5); /*send*/
	local[5]= w;
	local[6]= fqv[43];
	local[7]= fqv[45];
	ctx->vsp=local+8;
	w=(*ftab[0])(ctx,4,local+4,&ftab[0],fqv[20]); /*make-coords*/
	local[4]= w;
	local[5]= fqv[39];
	local[6]= loadglobal(fqv[21]);
	local[7]= fqv[40];
	local[8]= fqv[41];
	local[9]= fqv[42];
	ctx->vsp=local+10;
	w=(pointer)SEND(ctx,4,local+6); /*send*/
	local[6]= w;
	local[7]= fqv[44];
	local[8]= makeint((eusinteger_t)0L);
	local[9]= makeint((eusinteger_t)0L);
	local[10]= makeint((eusinteger_t)0L);
	ctx->vsp=local+11;
	w=(pointer)MKFLTVEC(ctx,3,local+8); /*float-vector*/
	local[8]= w;
	ctx->vsp=local+9;
	w=(pointer)SEND(ctx,3,local+6); /*send*/
	local[6]= w;
	local[7]= fqv[43];
	local[8]= fqv[40];
	ctx->vsp=local+9;
	w=(*ftab[0])(ctx,4,local+5,&ftab[0],fqv[20]); /*make-coords*/
	local[5]= w;
	ctx->vsp=local+6;
	w=(pointer)LIST(ctx,3,local+3); /*list*/
	local[3]= w;
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,3,local+1); /*send*/
	local[1]= loadglobal(fqv[37]);
	local[2]= fqv[46];
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,2,local+1); /*send*/
	local[1]= w;
	goto dribble_IF158;
dribble_IF157:
	local[1]= NIL;
dribble_IF158:
	local[1]= loadglobal(fqv[21]);
	local[2]= fqv[47];
	local[3]= fqv[48];
	local[4]= makeint((eusinteger_t)0L);
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,4,local+1); /*send*/
	local[1]= loadglobal(fqv[21]);
	local[2]= fqv[24];
	local[3]= fqv[48];
	local[4]= makeint((eusinteger_t)-45L);
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,4,local+1); /*send*/
	local[1]= loadglobal(fqv[21]);
	local[2]= fqv[25];
	local[3]= fqv[48];
	local[4]= makeint((eusinteger_t)0L);
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,4,local+1); /*send*/
	local[1]= loadglobal(fqv[21]);
	local[2]= fqv[49];
	local[3]= fqv[48];
	local[4]= makeint((eusinteger_t)-100L);
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,4,local+1); /*send*/
	local[1]= loadglobal(fqv[21]);
	local[2]= fqv[50];
	local[3]= fqv[48];
	local[4]= makeint((eusinteger_t)45L);
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,4,local+1); /*send*/
	local[1]= loadglobal(fqv[21]);
	local[2]= fqv[51];
	local[3]= fqv[48];
	local[4]= makeint((eusinteger_t)0L);
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,4,local+1); /*send*/
	local[1]= loadglobal(fqv[21]);
	local[2]= fqv[52];
	local[3]= fqv[48];
	local[4]= makeint((eusinteger_t)-25L);
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,4,local+1); /*send*/
	local[1]= loadglobal(fqv[21]);
	local[2]= fqv[53];
	local[3]= fqv[54];
	local[4]= fqv[55];
	local[5]= fqv[56];
	ctx->vsp=local+6;
	w=(pointer)SEND(ctx,5,local+1); /*send*/
	local[1]= loadglobal(fqv[21]);
	local[2]= fqv[57];
	local[3]= fqv[48];
	local[4]= makeint((eusinteger_t)20L);
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,4,local+1); /*send*/
	local[1]= loadglobal(fqv[21]);
	local[2]= fqv[58];
	local[3]= fqv[48];
	local[4]= makeint((eusinteger_t)0L);
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,4,local+1); /*send*/
	local[1]= loadglobal(fqv[21]);
	local[2]= fqv[59];
	local[3]= fqv[48];
	local[4]= makeint((eusinteger_t)0L);
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,4,local+1); /*send*/
	local[1]= loadglobal(fqv[37]);
	local[2]= fqv[60];
	local[3]= loadglobal(fqv[21]);
	local[4]= fqv[60];
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,2,local+3); /*send*/
	local[3]= w;
	local[4]= makeint((eusinteger_t)10000L);
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,4,local+1); /*send*/
	local[1]= loadglobal(fqv[37]);
	local[2]= fqv[46];
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,2,local+1); /*send*/
	local[0]= w;
dribble_BLK155:
	ctx->vsp=local; return(local[0]);}

/*:update-param*/
static pointer dribble_M159handtrajectorycontroller_update_param(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	ctx->vsp=local+0;
	n=parsekeyparams(fqv[61], &argv[2], n-2, local+0, 0);
	if (n & (1<<0)) goto dribble_KEY161;
	local[0] = NIL;
dribble_KEY161:
	if (n & (1<<1)) goto dribble_KEY162;
	local[1] = NIL;
dribble_KEY162:
	if (n & (1<<2)) goto dribble_KEY163;
	local[2] = NIL;
dribble_KEY163:
	if (n & (1<<3)) goto dribble_KEY164;
	local[3] = NIL;
dribble_KEY164:
	local[4]= argv[0];
	local[5]= fqv[62];
	local[6]= loadglobal(fqv[21]);
	local[7]= fqv[53];
	local[8]= fqv[41];
	local[9]= fqv[42];
	ctx->vsp=local+10;
	w=(pointer)SEND(ctx,4,local+6); /*send*/
	local[6]= w;
	local[7]= fqv[63];
	ctx->vsp=local+8;
	w=(pointer)SEND(ctx,2,local+6); /*send*/
	local[6]= w;
	local[7]= makeint((eusinteger_t)2L);
	ctx->vsp=local+8;
	w=(pointer)ELT(ctx,2,local+6); /*elt*/
	local[6]= w;
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,3,local+4); /*send*/
	local[4]= argv[0];
	local[5]= fqv[64];
	local[6]= loadglobal(fqv[21]);
	local[7]= fqv[53];
	local[8]= fqv[41];
	local[9]= fqv[42];
	ctx->vsp=local+10;
	w=(pointer)SEND(ctx,4,local+6); /*send*/
	local[6]= w;
	local[7]= fqv[65];
	ctx->vsp=local+8;
	w=(pointer)SEND(ctx,2,local+6); /*send*/
	if (!iscons(w) && w!=NIL) error(E_NOLIST);
	local[6]= (w)->c.cons.car;
	local[7]= loadglobal(fqv[66]);
	ctx->vsp=local+8;
	w=(pointer)COERCE(ctx,2,local+6); /*coerce*/
	local[6]= w;
	local[7]= makeint((eusinteger_t)1L);
	ctx->vsp=local+8;
	w=(pointer)ELT(ctx,2,local+6); /*elt*/
	local[6]= w;
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,3,local+4); /*send*/
	local[4]= argv[0];
	local[5]= fqv[67];
	local[6]= local[0];
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,3,local+4); /*send*/
	local[4]= argv[0];
	local[5]= fqv[68];
	local[6]= local[1];
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,3,local+4); /*send*/
	local[4]= argv[0];
	local[5]= fqv[69];
	local[6]= local[2];
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,3,local+4); /*send*/
	local[4]= argv[0];
	local[5]= fqv[70];
	local[6]= local[3];
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,3,local+4); /*send*/
	w = NIL;
	local[0]= w;
dribble_BLK160:
	ctx->vsp=local; return(local[0]);}

/*:make-target-hand-coords*/
static pointer dribble_M165handtrajectorycontroller_make_target_hand_coords(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	ctx->vsp=local+0;
	n=parsekeyparams(fqv[71], &argv[2], n-2, local+0, 0);
	if (n & (1<<0)) goto dribble_KEY167;
	local[0] = NIL;
dribble_KEY167:
	local[1]= argv[0];
	local[2]= fqv[72];
	local[3]= fqv[73];
	local[4]= local[0];
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,4,local+1); /*send*/
	local[1]= argv[0];
	local[2]= fqv[74];
	local[3]= fqv[73];
	local[4]= local[0];
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,4,local+1); /*send*/
	local[1]= argv[0];
	local[2]= fqv[75];
	local[3]= fqv[76];
	local[4]= argv[0];
	local[5]= fqv[77];
	ctx->vsp=local+6;
	w=(pointer)SEND(ctx,2,local+4); /*send*/
	local[4]= w;
	local[5]= fqv[78];
	local[6]= argv[0];
	local[7]= fqv[79];
	ctx->vsp=local+8;
	w=(pointer)SEND(ctx,2,local+6); /*send*/
	local[6]= w;
	ctx->vsp=local+7;
	w=(*ftab[0])(ctx,4,local+3,&ftab[0],fqv[20]); /*make-coords*/
	local[3]= w;
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,3,local+1); /*send*/
	local[0]= w;
dribble_BLK166:
	ctx->vsp=local; return(local[0]);}

/*:make-target-hand-pos*/
static pointer dribble_M168handtrajectorycontroller_make_target_hand_pos(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	ctx->vsp=local+0;
	n=parsekeyparams(fqv[80], &argv[2], n-2, local+0, 0);
	if (n & (1<<0)) goto dribble_KEY170;
	local[0] = NIL;
dribble_KEY170:
	local[1]= makeflt(5.0000000000000000000000e-01);
	local[2]= argv[0];
	local[3]= fqv[62];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	local[3]= argv[0];
	local[4]= fqv[67];
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,2,local+3); /*send*/
	local[3]= w;
	ctx->vsp=local+4;
	w=(pointer)MINUS(ctx,2,local+2); /*-*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)TIMES(ctx,2,local+1); /***/
	local[1]= w;
	local[2]= makeint((eusinteger_t)2L);
	local[3]= makeflt(3.1415926535897931159980e+00);
	ctx->vsp=local+4;
	w=(pointer)TIMES(ctx,2,local+2); /***/
	local[2]= w;
	local[3]= argv[0];
	local[4]= fqv[69];
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,2,local+3); /*send*/
	local[3]= w;
	ctx->vsp=local+4;
	w=(pointer)QUOTIENT(ctx,2,local+2); /*/*/
	local[2]= w;
	local[3]= makeflt(0.0000000000000000000000e+00);
	local[4]= argv[0];
	local[5]= fqv[62];
	ctx->vsp=local+6;
	w=(pointer)SEND(ctx,2,local+4); /*send*/
	local[4]= w;
	local[5]= argv[0];
	local[6]= fqv[67];
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,2,local+5); /*send*/
	local[5]= w;
	ctx->vsp=local+6;
	w=(pointer)PLUS(ctx,2,local+4); /*+*/
	local[4]= w;
	local[5]= makeflt(2.0000000000000000000000e+00);
	ctx->vsp=local+6;
	w=(pointer)QUOTIENT(ctx,2,local+4); /*/*/
	local[4]= w;
	local[5]= loadglobal(fqv[21]);
	local[6]= fqv[53];
	local[7]= fqv[41];
	local[8]= fqv[42];
	ctx->vsp=local+9;
	w=(pointer)SEND(ctx,4,local+5); /*send*/
	local[5]= w;
	local[6]= argv[0]->c.obj.iv[12];
	local[7]= makeint((eusinteger_t)0L);
	local[8]= local[5];
	local[9]= fqv[76];
	ctx->vsp=local+10;
	w=(pointer)SEND(ctx,2,local+8); /*send*/
	local[8]= w;
	local[9]= makeint((eusinteger_t)0L);
	ctx->vsp=local+10;
	w=(pointer)ELT(ctx,2,local+8); /*elt*/
	local[8]= w;
	ctx->vsp=local+9;
	w=(pointer)ASET(ctx,3,local+6); /*aset*/
	local[6]= argv[0]->c.obj.iv[12];
	local[7]= makeint((eusinteger_t)1L);
	local[8]= local[5];
	local[9]= fqv[76];
	ctx->vsp=local+10;
	w=(pointer)SEND(ctx,2,local+8); /*send*/
	local[8]= w;
	local[9]= makeint((eusinteger_t)1L);
	ctx->vsp=local+10;
	w=(pointer)ELT(ctx,2,local+8); /*elt*/
	local[8]= w;
	ctx->vsp=local+9;
	w=(pointer)ASET(ctx,3,local+6); /*aset*/
	local[6]= argv[0]->c.obj.iv[12];
	local[7]= makeint((eusinteger_t)2L);
	local[8]= local[1];
	local[9]= local[2];
	local[10]= loadglobal(fqv[19]);
	ctx->vsp=local+11;
	w=(pointer)TIMES(ctx,2,local+9); /***/
	local[9]= w;
	local[10]= local[3];
	ctx->vsp=local+11;
	w=(pointer)PLUS(ctx,2,local+9); /*+*/
	local[9]= w;
	ctx->vsp=local+10;
	w=(pointer)COS(ctx,1,local+9); /*cos*/
	local[9]= w;
	ctx->vsp=local+10;
	w=(pointer)TIMES(ctx,2,local+8); /***/
	local[8]= w;
	local[9]= local[4];
	ctx->vsp=local+10;
	w=(pointer)PLUS(ctx,2,local+8); /*+*/
	local[8]= w;
	ctx->vsp=local+9;
	w=(pointer)ASET(ctx,3,local+6); /*aset*/
	local[0]= w;
dribble_BLK169:
	ctx->vsp=local; return(local[0]);}

/*:make-target-hand-rpy*/
static pointer dribble_M171handtrajectorycontroller_make_target_hand_rpy(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	ctx->vsp=local+0;
	n=parsekeyparams(fqv[81], &argv[2], n-2, local+0, 0);
	if (n & (1<<0)) goto dribble_KEY173;
	local[0] = NIL;
dribble_KEY173:
	local[1]= makeflt(5.0000000000000000000000e-01);
	local[2]= argv[0];
	local[3]= fqv[68];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	local[3]= argv[0];
	local[4]= fqv[64];
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,2,local+3); /*send*/
	local[3]= w;
	ctx->vsp=local+4;
	w=(pointer)MINUS(ctx,2,local+2); /*-*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)TIMES(ctx,2,local+1); /***/
	local[1]= w;
	local[2]= makeint((eusinteger_t)2L);
	local[3]= makeflt(3.1415926535897931159980e+00);
	ctx->vsp=local+4;
	w=(pointer)TIMES(ctx,2,local+2); /***/
	local[2]= w;
	local[3]= argv[0];
	local[4]= fqv[69];
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,2,local+3); /*send*/
	local[3]= w;
	ctx->vsp=local+4;
	w=(pointer)QUOTIENT(ctx,2,local+2); /*/*/
	local[2]= w;
	local[3]= makeflt(3.1415926535897931159980e+00);
	local[4]= argv[0];
	local[5]= fqv[64];
	ctx->vsp=local+6;
	w=(pointer)SEND(ctx,2,local+4); /*send*/
	local[4]= w;
	local[5]= argv[0];
	local[6]= fqv[68];
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,2,local+5); /*send*/
	local[5]= w;
	ctx->vsp=local+6;
	w=(pointer)PLUS(ctx,2,local+4); /*+*/
	local[4]= w;
	local[5]= makeflt(2.0000000000000000000000e+00);
	ctx->vsp=local+6;
	w=(pointer)QUOTIENT(ctx,2,local+4); /*/*/
	local[4]= w;
	local[5]= loadglobal(fqv[21]);
	local[6]= fqv[53];
	local[7]= fqv[41];
	local[8]= fqv[42];
	ctx->vsp=local+9;
	w=(pointer)SEND(ctx,4,local+5); /*send*/
	local[5]= w;
	local[6]= argv[0]->c.obj.iv[13];
	local[7]= makeint((eusinteger_t)0L);
	local[8]= local[5];
	local[9]= fqv[65];
	ctx->vsp=local+10;
	w=(pointer)SEND(ctx,2,local+8); /*send*/
	if (!iscons(w) && w!=NIL) error(E_NOLIST);
	local[8]= (w)->c.cons.car;
	local[9]= makeint((eusinteger_t)0L);
	ctx->vsp=local+10;
	w=(pointer)ELT(ctx,2,local+8); /*elt*/
	local[8]= w;
	ctx->vsp=local+9;
	w=(pointer)ASET(ctx,3,local+6); /*aset*/
	local[6]= argv[0]->c.obj.iv[13];
	local[7]= makeint((eusinteger_t)1L);
	local[8]= local[1];
	local[9]= local[2];
	local[10]= loadglobal(fqv[19]);
	ctx->vsp=local+11;
	w=(pointer)TIMES(ctx,2,local+9); /***/
	local[9]= w;
	local[10]= local[3];
	ctx->vsp=local+11;
	w=(pointer)PLUS(ctx,2,local+9); /*+*/
	local[9]= w;
	ctx->vsp=local+10;
	w=(pointer)COS(ctx,1,local+9); /*cos*/
	local[9]= w;
	ctx->vsp=local+10;
	w=(pointer)TIMES(ctx,2,local+8); /***/
	local[8]= w;
	local[9]= local[4];
	ctx->vsp=local+10;
	w=(pointer)PLUS(ctx,2,local+8); /*+*/
	local[8]= w;
	ctx->vsp=local+9;
	w=(pointer)ASET(ctx,3,local+6); /*aset*/
	local[6]= argv[0]->c.obj.iv[13];
	local[7]= makeint((eusinteger_t)2L);
	local[8]= local[5];
	local[9]= fqv[65];
	ctx->vsp=local+10;
	w=(pointer)SEND(ctx,2,local+8); /*send*/
	if (!iscons(w) && w!=NIL) error(E_NOLIST);
	local[8]= (w)->c.cons.car;
	local[9]= makeint((eusinteger_t)2L);
	ctx->vsp=local+10;
	w=(pointer)ELT(ctx,2,local+8); /*elt*/
	local[8]= w;
	ctx->vsp=local+9;
	w=(pointer)ASET(ctx,3,local+6); /*aset*/
	local[0]= w;
dribble_BLK172:
	ctx->vsp=local; return(local[0]);}

/*:solve-ik*/
static pointer dribble_M174handtrajectorycontroller_solve_ik(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	ctx->vsp=local+0;
	n=parsekeyparams(fqv[82], &argv[2], n-2, local+0, 0);
	if (n & (1<<0)) goto dribble_KEY176;
	local[0] = T;
dribble_KEY176:
	if (n & (1<<1)) goto dribble_KEY177;
	local[1] = T;
dribble_KEY177:
	local[2]= argv[0];
	local[3]= fqv[75];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	if (NIL!=local[2]) goto dribble_IF178;
	local[2]= argv[0];
	local[3]= fqv[83];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	goto dribble_IF179;
dribble_IF178:
	local[2]= NIL;
dribble_IF179:
	local[2]= loadglobal(fqv[21]);
	local[3]= fqv[53];
	local[4]= fqv[84];
	local[5]= argv[0];
	local[6]= fqv[75];
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,2,local+5); /*send*/
	local[5]= w;
	local[6]= fqv[85];
	local[7]= makeflt(9.9999999999999950039964e-03);
	local[8]= fqv[86];
	local[9]= makeflt(9.9999999999999950039964e-03);
	local[10]= fqv[87];
	local[11]= argv[0]->c.obj.iv[21];
	local[12]= fqv[88];
	local[13]= T;
	local[14]= fqv[89];
	local[15]= T;
	ctx->vsp=local+16;
	w=(pointer)SEND(ctx,14,local+2); /*send*/
	local[0]= w;
dribble_BLK175:
	ctx->vsp=local; return(local[0]);}

/*:dribble-mode*/
static pointer dribble_M180handtrajectorycontroller_dribble_mode(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	ctx->vsp=local+0;
	n=parsekeyparams(fqv[90], &argv[2], n-2, local+0, 0);
	if (n & (1<<0)) goto dribble_KEY182;
	local[0] = NIL;
dribble_KEY182:
	local[1]= argv[0];
	local[2]= fqv[91];
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,2,local+1); /*send*/
	local[1]= argv[0];
	local[2]= fqv[92];
	local[3]= makeflt(0.0000000000000000000000e+00);
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,3,local+1); /*send*/
	local[1]= loadglobal(fqv[29]);
	ctx->vsp=local+2;
	w=(pointer)INSTANTIATE(ctx,1,local+1); /*instantiate*/
	local[1]= w;
	local[2]= local[1];
	local[3]= fqv[93];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	w = local[1];
	argv[0]->c.obj.iv[3] = w;
	local[1]= argv[0]->c.obj.iv[3];
	local[2]= fqv[43];
	local[3]= loadglobal(fqv[94]);
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,3,local+1); /*send*/
	local[1]= T;
	local[2]= fqv[95];
	ctx->vsp=local+3;
	w=(pointer)XFORMAT(ctx,2,local+1); /*format*/
	local[0]= w;
dribble_BLK181:
	ctx->vsp=local; return(local[0]);}

/*:motion-cb*/
static pointer dribble_M183handtrajectorycontroller_motion_cb(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=3) maerror();
	local[0]= fqv[96];
	local[1]= loadglobal(fqv[18]);
	ctx->vsp=local+2;
	w=(*ftab[3])(ctx,2,local+0,&ftab[3],fqv[97]); /*ros::publish*/
	local[0]= w;
dribble_BLK184:
	ctx->vsp=local; return(local[0]);}

/*:start-motion*/
static pointer dribble_M185handtrajectorycontroller_start_motion(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=2) maerror();
	local[0]= makeint((eusinteger_t)500L);
	ctx->vsp=local+1;
	w=(*ftab[4])(ctx,1,local+0,&ftab[4],fqv[98]); /*ros::rate*/
dribble_WHL187:
	ctx->vsp=local+0;
	w=(*ftab[5])(ctx,0,local+0,&ftab[5],fqv[99]); /*ros::ok*/
	if (w==NIL) goto dribble_WHX188;
	local[0]= fqv[100];
	local[1]= loadglobal(fqv[18]);
	ctx->vsp=local+2;
	w=(*ftab[3])(ctx,2,local+0,&ftab[3],fqv[97]); /*ros::publish*/
	ctx->vsp=local+0;
	w=(*ftab[6])(ctx,0,local+0,&ftab[6],fqv[101]); /*ros::sleep*/
	goto dribble_WHL187;
dribble_WHX188:
	local[0]= NIL;
dribble_BLK189:
	w = local[0];
	local[0]= w;
dribble_BLK186:
	ctx->vsp=local; return(local[0]);}

/*:start-motion2*/
static pointer dribble_M190handtrajectorycontroller_start_motion2(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=2) maerror();
	local[0]= makeflt(2.0000000000000000416334e-03);
	local[1]= (pointer)get_sym_func(fqv[102]);
	local[2]= argv[0];
	local[3]= fqv[103];
	ctx->vsp=local+4;
	w=(*ftab[7])(ctx,4,local+0,&ftab[7],fqv[104]); /*ros::create-timer*/
	local[0]= makeint((eusinteger_t)500L);
	ctx->vsp=local+1;
	w=(*ftab[4])(ctx,1,local+0,&ftab[4],fqv[98]); /*ros::rate*/
	ctx->vsp=local+0;
	w=(*ftab[8])(ctx,0,local+0,&ftab[8],fqv[105]); /*ros::spin*/
	local[0]= w;
dribble_BLK191:
	ctx->vsp=local; return(local[0]);}

/*:update-param2*/
static pointer dribble_M192handtrajectorycontroller_update_param2(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	ctx->vsp=local+0;
	n=parsekeyparams(fqv[106], &argv[2], n-2, local+0, 0);
	if (n & (1<<0)) goto dribble_KEY194;
	local[0] = NIL;
dribble_KEY194:
	if (n & (1<<1)) goto dribble_KEY195;
	local[1] = NIL;
dribble_KEY195:
	if (n & (1<<2)) goto dribble_KEY196;
	local[2] = NIL;
dribble_KEY196:
	if (n & (1<<3)) goto dribble_KEY197;
	local[3] = NIL;
dribble_KEY197:
	local[4]= loadglobal(fqv[21]);
	local[5]= fqv[60];
	local[6]= argv[0];
	local[7]= fqv[107];
	ctx->vsp=local+8;
	w=(pointer)SEND(ctx,2,local+6); /*send*/
	local[6]= w;
	ctx->vsp=local+7;
	w=(*ftab[9])(ctx,1,local+6,&ftab[9],fqv[108]); /*last*/
	if (!iscons(w) && w!=NIL) error(E_NOLIST);
	local[6]= (w)->c.cons.car;
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,3,local+4); /*send*/
	local[4]= argv[0];
	local[5]= fqv[109];
	local[6]= loadglobal(fqv[21]);
	local[7]= fqv[60];
	ctx->vsp=local+8;
	w=(pointer)SEND(ctx,2,local+6); /*send*/
	local[6]= w;
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,3,local+4); /*send*/
	local[4]= argv[0];
	local[5]= fqv[62];
	local[6]= loadglobal(fqv[21]);
	local[7]= fqv[53];
	local[8]= fqv[41];
	local[9]= fqv[42];
	ctx->vsp=local+10;
	w=(pointer)SEND(ctx,4,local+6); /*send*/
	local[6]= w;
	local[7]= fqv[63];
	ctx->vsp=local+8;
	w=(pointer)SEND(ctx,2,local+6); /*send*/
	local[6]= w;
	local[7]= makeint((eusinteger_t)2L);
	ctx->vsp=local+8;
	w=(pointer)ELT(ctx,2,local+6); /*elt*/
	local[6]= w;
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,3,local+4); /*send*/
	local[4]= argv[0];
	local[5]= fqv[64];
	local[6]= loadglobal(fqv[21]);
	local[7]= fqv[53];
	local[8]= fqv[41];
	local[9]= fqv[42];
	ctx->vsp=local+10;
	w=(pointer)SEND(ctx,4,local+6); /*send*/
	local[6]= w;
	local[7]= fqv[65];
	ctx->vsp=local+8;
	w=(pointer)SEND(ctx,2,local+6); /*send*/
	if (!iscons(w) && w!=NIL) error(E_NOLIST);
	local[6]= (w)->c.cons.car;
	local[7]= loadglobal(fqv[66]);
	ctx->vsp=local+8;
	w=(pointer)COERCE(ctx,2,local+6); /*coerce*/
	local[6]= w;
	local[7]= makeint((eusinteger_t)1L);
	ctx->vsp=local+8;
	w=(pointer)ELT(ctx,2,local+6); /*elt*/
	local[6]= w;
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,3,local+4); /*send*/
	local[4]= argv[0];
	local[5]= fqv[67];
	local[6]= local[0];
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,3,local+4); /*send*/
	local[4]= argv[0];
	local[5]= fqv[68];
	local[6]= local[1];
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,3,local+4); /*send*/
	local[4]= argv[0];
	local[5]= fqv[69];
	local[6]= local[2];
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,3,local+4); /*send*/
	local[4]= argv[0];
	local[5]= fqv[70];
	local[6]= local[3];
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,3,local+4); /*send*/
	w = NIL;
	local[0]= w;
dribble_BLK193:
	ctx->vsp=local; return(local[0]);}

/*:make-dt-list*/
static pointer dribble_M198handtrajectorycontroller_make_dt_list(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=2) maerror();
	local[0]= argv[0];
	local[1]= fqv[110];
	local[2]= argv[0];
	local[3]= fqv[69];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	local[3]= argv[0];
	local[4]= fqv[70];
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,2,local+3); /*send*/
	local[3]= w;
	ctx->vsp=local+4;
	w=(pointer)TIMES(ctx,2,local+2); /***/
	local[2]= w;
	local[3]= makeflt(2.0000000000000000000000e+00);
	ctx->vsp=local+4;
	w=(pointer)QUOTIENT(ctx,2,local+2); /*/*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)ROUND(ctx,1,local+2); /*round*/
	local[2]= w;
	local[3]= fqv[111];
	local[4]= makeflt(2.0000000000000000000000e+00);
	ctx->vsp=local+5;
	w=(*ftab[10])(ctx,3,local+2,&ftab[10],fqv[112]); /*make-list*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,3,local+0); /*send*/
	local[0]= w;
dribble_BLK199:
	ctx->vsp=local; return(local[0]);}

/*:make-target-hand-coords-list*/
static pointer dribble_M200handtrajectorycontroller_make_target_hand_coords_list(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=2) maerror();
	local[0]= argv[0];
	local[1]= fqv[113];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= argv[0];
	local[1]= fqv[114];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= NIL;
	local[1]= NIL;
	local[2]= makeint((eusinteger_t)0L);
	local[3]= argv[0];
	local[4]= fqv[110];
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,2,local+3); /*send*/
	local[3]= w;
	ctx->vsp=local+4;
	w=(pointer)LENGTH(ctx,1,local+3); /*length*/
	local[3]= w;
dribble_WHL202:
	local[4]= local[2];
	w = local[3];
	if ((eusinteger_t)local[4] >= (eusinteger_t)w) goto dribble_WHX203;
	local[4]= fqv[76];
	local[5]= argv[0];
	local[6]= fqv[115];
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,2,local+5); /*send*/
	local[5]= w;
	local[6]= local[2];
	ctx->vsp=local+7;
	w=(pointer)ELT(ctx,2,local+5); /*elt*/
	local[5]= w;
	local[6]= fqv[78];
	local[7]= argv[0];
	local[8]= fqv[116];
	ctx->vsp=local+9;
	w=(pointer)SEND(ctx,2,local+7); /*send*/
	local[7]= w;
	local[8]= local[2];
	ctx->vsp=local+9;
	w=(pointer)ELT(ctx,2,local+7); /*elt*/
	local[7]= w;
	ctx->vsp=local+8;
	w=(*ftab[0])(ctx,4,local+4,&ftab[0],fqv[20]); /*make-coords*/
	local[0] = w;
	local[4]= local[0];
	w = local[1];
	ctx->vsp=local+5;
	local[1] = cons(ctx,local[4],w);
	local[4]= local[2];
	ctx->vsp=local+5;
	w=(pointer)ADD1(ctx,1,local+4); /*1+*/
	local[2] = w;
	goto dribble_WHL202;
dribble_WHX203:
	local[4]= NIL;
dribble_BLK204:
	w = NIL;
	local[2]= local[1];
	ctx->vsp=local+3;
	w=(pointer)NREVERSE(ctx,1,local+2); /*nreverse*/
	local[2]= argv[0];
	local[3]= fqv[117];
	local[4]= local[1];
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,3,local+2); /*send*/
	w = NIL;
	local[0]= w;
dribble_BLK201:
	ctx->vsp=local; return(local[0]);}

/*:make-target-hand-pos-list*/
static pointer dribble_M205handtrajectorycontroller_make_target_hand_pos_list(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=2) maerror();
	local[0]= argv[0];
	local[1]= fqv[110];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= w;
	if (NIL!=local[0]) goto dribble_IF207;
	local[0]= argv[0];
	local[1]= fqv[118];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= w;
	goto dribble_IF208;
dribble_IF207:
	local[0]= NIL;
dribble_IF208:
	local[0]= makeflt(5.0000000000000000000000e-01);
	local[1]= argv[0];
	local[2]= fqv[62];
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,2,local+1); /*send*/
	local[1]= w;
	local[2]= argv[0];
	local[3]= fqv[67];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)MINUS(ctx,2,local+1); /*-*/
	local[1]= w;
	ctx->vsp=local+2;
	w=(pointer)TIMES(ctx,2,local+0); /***/
	local[0]= w;
	local[1]= makeint((eusinteger_t)2L);
	local[2]= makeflt(3.1415926535897931159980e+00);
	ctx->vsp=local+3;
	w=(pointer)TIMES(ctx,2,local+1); /***/
	local[1]= w;
	local[2]= argv[0];
	local[3]= fqv[69];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)QUOTIENT(ctx,2,local+1); /*/*/
	local[1]= w;
	local[2]= makeflt(0.0000000000000000000000e+00);
	local[3]= argv[0];
	local[4]= fqv[62];
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,2,local+3); /*send*/
	local[3]= w;
	local[4]= argv[0];
	local[5]= fqv[67];
	ctx->vsp=local+6;
	w=(pointer)SEND(ctx,2,local+4); /*send*/
	local[4]= w;
	ctx->vsp=local+5;
	w=(pointer)PLUS(ctx,2,local+3); /*+*/
	local[3]= w;
	local[4]= makeflt(2.0000000000000000000000e+00);
	ctx->vsp=local+5;
	w=(pointer)QUOTIENT(ctx,2,local+3); /*/*/
	local[3]= w;
	local[4]= NIL;
	local[5]= NIL;
	local[6]= loadglobal(fqv[21]);
	local[7]= fqv[53];
	local[8]= fqv[41];
	local[9]= fqv[42];
	ctx->vsp=local+10;
	w=(pointer)SEND(ctx,4,local+6); /*send*/
	local[6]= w;
	local[7]= makeint((eusinteger_t)0L);
	local[8]= argv[0];
	local[9]= fqv[110];
	ctx->vsp=local+10;
	w=(pointer)SEND(ctx,2,local+8); /*send*/
	local[8]= w;
	ctx->vsp=local+9;
	w=(pointer)LENGTH(ctx,1,local+8); /*length*/
	local[8]= w;
dribble_WHL209:
	local[9]= local[7];
	w = local[8];
	if ((eusinteger_t)local[9] >= (eusinteger_t)w) goto dribble_WHX210;
	local[9]= local[6];
	local[10]= fqv[76];
	ctx->vsp=local+11;
	w=(pointer)SEND(ctx,2,local+9); /*send*/
	local[9]= w;
	local[10]= makeint((eusinteger_t)0L);
	ctx->vsp=local+11;
	w=(pointer)ELT(ctx,2,local+9); /*elt*/
	local[9]= w;
	local[10]= local[6];
	local[11]= fqv[76];
	ctx->vsp=local+12;
	w=(pointer)SEND(ctx,2,local+10); /*send*/
	local[10]= w;
	local[11]= makeint((eusinteger_t)1L);
	ctx->vsp=local+12;
	w=(pointer)ELT(ctx,2,local+10); /*elt*/
	local[10]= w;
	local[11]= local[0];
	local[12]= local[1];
	local[13]= argv[0]->c.obj.iv[11];
	local[14]= local[7];
	ctx->vsp=local+15;
	w=(pointer)TIMES(ctx,2,local+13); /***/
	local[13]= w;
	ctx->vsp=local+14;
	w=(pointer)TIMES(ctx,2,local+12); /***/
	local[12]= w;
	local[13]= local[2];
	ctx->vsp=local+14;
	w=(pointer)PLUS(ctx,2,local+12); /*+*/
	local[12]= w;
	ctx->vsp=local+13;
	w=(pointer)COS(ctx,1,local+12); /*cos*/
	local[12]= w;
	ctx->vsp=local+13;
	w=(pointer)TIMES(ctx,2,local+11); /***/
	local[11]= w;
	local[12]= local[3];
	ctx->vsp=local+13;
	w=(pointer)PLUS(ctx,2,local+11); /*+*/
	local[11]= w;
	ctx->vsp=local+12;
	w=(pointer)MKFLTVEC(ctx,3,local+9); /*float-vector*/
	local[4] = w;
	local[9]= local[4];
	w = local[5];
	ctx->vsp=local+10;
	local[5] = cons(ctx,local[9],w);
	local[9]= local[7];
	ctx->vsp=local+10;
	w=(pointer)ADD1(ctx,1,local+9); /*1+*/
	local[7] = w;
	goto dribble_WHL209;
dribble_WHX210:
	local[9]= NIL;
dribble_BLK211:
	w = NIL;
	local[7]= local[5];
	ctx->vsp=local+8;
	w=(pointer)NREVERSE(ctx,1,local+7); /*nreverse*/
	local[7]= argv[0];
	local[8]= fqv[115];
	local[9]= local[5];
	ctx->vsp=local+10;
	w=(pointer)SEND(ctx,3,local+7); /*send*/
	local[0]= w;
dribble_BLK206:
	ctx->vsp=local; return(local[0]);}

/*:make-target-hand-rpy-list*/
static pointer dribble_M212handtrajectorycontroller_make_target_hand_rpy_list(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=2) maerror();
	local[0]= argv[0];
	local[1]= fqv[110];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= w;
	if (NIL!=local[0]) goto dribble_IF214;
	local[0]= argv[0];
	local[1]= fqv[118];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= w;
	goto dribble_IF215;
dribble_IF214:
	local[0]= NIL;
dribble_IF215:
	local[0]= makeflt(5.0000000000000000000000e-01);
	local[1]= argv[0];
	local[2]= fqv[68];
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,2,local+1); /*send*/
	local[1]= w;
	local[2]= argv[0];
	local[3]= fqv[64];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)MINUS(ctx,2,local+1); /*-*/
	local[1]= w;
	ctx->vsp=local+2;
	w=(pointer)TIMES(ctx,2,local+0); /***/
	local[0]= w;
	local[1]= makeint((eusinteger_t)2L);
	local[2]= makeflt(3.1415926535897931159980e+00);
	ctx->vsp=local+3;
	w=(pointer)TIMES(ctx,2,local+1); /***/
	local[1]= w;
	local[2]= argv[0];
	local[3]= fqv[69];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)QUOTIENT(ctx,2,local+1); /*/*/
	local[1]= w;
	local[2]= makeflt(3.1415926535897931159980e+00);
	local[3]= argv[0];
	local[4]= fqv[64];
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,2,local+3); /*send*/
	local[3]= w;
	local[4]= argv[0];
	local[5]= fqv[68];
	ctx->vsp=local+6;
	w=(pointer)SEND(ctx,2,local+4); /*send*/
	local[4]= w;
	ctx->vsp=local+5;
	w=(pointer)PLUS(ctx,2,local+3); /*+*/
	local[3]= w;
	local[4]= makeflt(2.0000000000000000000000e+00);
	ctx->vsp=local+5;
	w=(pointer)QUOTIENT(ctx,2,local+3); /*/*/
	local[3]= w;
	local[4]= NIL;
	local[5]= NIL;
	local[6]= loadglobal(fqv[21]);
	local[7]= fqv[53];
	local[8]= fqv[41];
	local[9]= fqv[42];
	ctx->vsp=local+10;
	w=(pointer)SEND(ctx,4,local+6); /*send*/
	local[6]= w;
	local[7]= makeint((eusinteger_t)0L);
	local[8]= argv[0];
	local[9]= fqv[110];
	ctx->vsp=local+10;
	w=(pointer)SEND(ctx,2,local+8); /*send*/
	local[8]= w;
	ctx->vsp=local+9;
	w=(pointer)LENGTH(ctx,1,local+8); /*length*/
	local[8]= w;
dribble_WHL216:
	local[9]= local[7];
	w = local[8];
	if ((eusinteger_t)local[9] >= (eusinteger_t)w) goto dribble_WHX217;
	local[9]= local[6];
	local[10]= fqv[65];
	ctx->vsp=local+11;
	w=(pointer)SEND(ctx,2,local+9); /*send*/
	if (!iscons(w) && w!=NIL) error(E_NOLIST);
	local[9]= (w)->c.cons.car;
	local[10]= makeint((eusinteger_t)0L);
	ctx->vsp=local+11;
	w=(pointer)ELT(ctx,2,local+9); /*elt*/
	local[9]= w;
	local[10]= local[0];
	local[11]= local[1];
	local[12]= argv[0]->c.obj.iv[11];
	local[13]= local[7];
	ctx->vsp=local+14;
	w=(pointer)TIMES(ctx,2,local+12); /***/
	local[12]= w;
	ctx->vsp=local+13;
	w=(pointer)TIMES(ctx,2,local+11); /***/
	local[11]= w;
	local[12]= local[2];
	ctx->vsp=local+13;
	w=(pointer)PLUS(ctx,2,local+11); /*+*/
	local[11]= w;
	ctx->vsp=local+12;
	w=(pointer)COS(ctx,1,local+11); /*cos*/
	local[11]= w;
	ctx->vsp=local+12;
	w=(pointer)TIMES(ctx,2,local+10); /***/
	local[10]= w;
	local[11]= local[3];
	ctx->vsp=local+12;
	w=(pointer)PLUS(ctx,2,local+10); /*+*/
	local[10]= w;
	local[11]= local[6];
	local[12]= fqv[65];
	ctx->vsp=local+13;
	w=(pointer)SEND(ctx,2,local+11); /*send*/
	if (!iscons(w) && w!=NIL) error(E_NOLIST);
	local[11]= (w)->c.cons.car;
	local[12]= makeint((eusinteger_t)2L);
	ctx->vsp=local+13;
	w=(pointer)ELT(ctx,2,local+11); /*elt*/
	local[11]= w;
	ctx->vsp=local+12;
	w=(pointer)MKFLTVEC(ctx,3,local+9); /*float-vector*/
	local[4] = w;
	local[9]= local[4];
	w = local[5];
	ctx->vsp=local+10;
	local[5] = cons(ctx,local[9],w);
	local[9]= local[7];
	ctx->vsp=local+10;
	w=(pointer)ADD1(ctx,1,local+9); /*1+*/
	local[7] = w;
	goto dribble_WHL216;
dribble_WHX217:
	local[9]= NIL;
dribble_BLK218:
	w = NIL;
	local[7]= local[5];
	ctx->vsp=local+8;
	w=(pointer)NREVERSE(ctx,1,local+7); /*nreverse*/
	local[7]= argv[0];
	local[8]= fqv[116];
	local[9]= local[5];
	ctx->vsp=local+10;
	w=(pointer)SEND(ctx,3,local+7); /*send*/
	local[0]= w;
dribble_BLK213:
	ctx->vsp=local; return(local[0]);}

/*:solve-ik2*/
static pointer dribble_M219handtrajectorycontroller_solve_ik2(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	ctx->vsp=local+0;
	n=parsekeyparams(fqv[119], &argv[2], n-2, local+0, 0);
	if (n & (1<<0)) goto dribble_KEY221;
	local[0] = T;
dribble_KEY221:
	if (n & (1<<1)) goto dribble_KEY222;
	local[1] = T;
dribble_KEY222:
	local[2]= argv[0];
	local[3]= fqv[117];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	if (NIL!=local[2]) goto dribble_IF223;
	local[2]= argv[0];
	local[3]= fqv[120];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	goto dribble_IF224;
dribble_IF223:
	local[2]= NIL;
dribble_IF224:
	local[2]= NIL;
	local[3]= NIL;
	local[4]= NIL;
	local[5]= makeint((eusinteger_t)0L);
	local[6]= argv[0];
	local[7]= fqv[110];
	ctx->vsp=local+8;
	w=(pointer)SEND(ctx,2,local+6); /*send*/
	local[6]= w;
	ctx->vsp=local+7;
	w=(pointer)LENGTH(ctx,1,local+6); /*length*/
	local[6]= w;
dribble_WHL225:
	local[7]= local[5];
	w = local[6];
	if ((eusinteger_t)local[7] >= (eusinteger_t)w) goto dribble_WHX226;
	local[7]= loadglobal(fqv[21]);
	local[8]= fqv[53];
	local[9]= fqv[84];
	local[10]= argv[0];
	local[11]= fqv[117];
	ctx->vsp=local+12;
	w=(pointer)SEND(ctx,2,local+10); /*send*/
	local[10]= w;
	local[11]= local[5];
	ctx->vsp=local+12;
	w=(pointer)ELT(ctx,2,local+10); /*elt*/
	local[10]= w;
	local[11]= fqv[85];
	local[12]= makeflt(5.0000000000000000000000e-01);
	local[13]= fqv[86];
	local[14]= makeflt(9.9999999999999950039964e-03);
	local[15]= fqv[87];
	local[16]= loadglobal(fqv[21]);
	local[17]= fqv[22];
	local[18]= fqv[23];
	ctx->vsp=local+19;
	w=(pointer)SEND(ctx,3,local+16); /*send*/
	local[16]= w;
	local[17]= makeflt(9.9999999999999977795540e-02);
	ctx->vsp=local+18;
	w=(pointer)LIST(ctx,2,local+16); /*list*/
	local[16]= w;
	local[17]= loadglobal(fqv[21]);
	local[18]= fqv[24];
	local[19]= fqv[23];
	ctx->vsp=local+20;
	w=(pointer)SEND(ctx,3,local+17); /*send*/
	local[17]= w;
	local[18]= makeflt(9.9999999999999950039964e-03);
	ctx->vsp=local+19;
	w=(pointer)LIST(ctx,2,local+17); /*list*/
	local[17]= w;
	local[18]= loadglobal(fqv[21]);
	local[19]= fqv[25];
	local[20]= fqv[23];
	ctx->vsp=local+21;
	w=(pointer)SEND(ctx,3,local+18); /*send*/
	local[18]= w;
	local[19]= makeflt(9.9999999999999977795540e-02);
	ctx->vsp=local+20;
	w=(pointer)LIST(ctx,2,local+18); /*list*/
	local[18]= w;
	ctx->vsp=local+19;
	w=(pointer)LIST(ctx,3,local+16); /*list*/
	local[16]= w;
	local[17]= fqv[88];
	local[18]= local[0];
	local[19]= fqv[89];
	local[20]= local[1];
	ctx->vsp=local+21;
	w=(pointer)SEND(ctx,14,local+7); /*send*/
	local[7]= loadglobal(fqv[21]);
	local[8]= fqv[60];
	ctx->vsp=local+9;
	w=(pointer)SEND(ctx,2,local+7); /*send*/
	local[7]= w;
	w = local[2];
	ctx->vsp=local+8;
	local[2] = cons(ctx,local[7],w);
	local[7]= loadglobal(fqv[21]);
	local[8]= fqv[53];
	local[9]= fqv[41];
	local[10]= fqv[42];
	ctx->vsp=local+11;
	w=(pointer)SEND(ctx,4,local+7); /*send*/
	local[7]= w;
	local[8]= fqv[63];
	ctx->vsp=local+9;
	w=(pointer)SEND(ctx,2,local+7); /*send*/
	local[7]= w;
	w = local[3];
	ctx->vsp=local+8;
	local[3] = cons(ctx,local[7],w);
	local[7]= loadglobal(fqv[21]);
	local[8]= fqv[53];
	local[9]= fqv[41];
	local[10]= fqv[42];
	ctx->vsp=local+11;
	w=(pointer)SEND(ctx,4,local+7); /*send*/
	local[7]= w;
	local[8]= fqv[65];
	ctx->vsp=local+9;
	w=(pointer)SEND(ctx,2,local+7); /*send*/
	if (!iscons(w) && w!=NIL) error(E_NOLIST);
	local[7]= (w)->c.cons.car;
	w = local[4];
	ctx->vsp=local+8;
	local[4] = cons(ctx,local[7],w);
	local[7]= local[5];
	ctx->vsp=local+8;
	w=(pointer)ADD1(ctx,1,local+7); /*1+*/
	local[5] = w;
	goto dribble_WHL225;
dribble_WHX226:
	local[7]= NIL;
dribble_BLK227:
	w = NIL;
	local[5]= local[2];
	ctx->vsp=local+6;
	w=(pointer)NREVERSE(ctx,1,local+5); /*nreverse*/
	local[5]= local[3];
	ctx->vsp=local+6;
	w=(pointer)NREVERSE(ctx,1,local+5); /*nreverse*/
	local[5]= local[4];
	ctx->vsp=local+6;
	w=(pointer)NREVERSE(ctx,1,local+5); /*nreverse*/
	local[5]= argv[0];
	local[6]= fqv[107];
	local[7]= local[2];
	ctx->vsp=local+8;
	w=(pointer)SEND(ctx,3,local+5); /*send*/
	storeglobal(fqv[121],local[3]);
	local[5]= local[4];
	storeglobal(fqv[122],local[5]);
	w = local[5];
	w = NIL;
	local[0]= w;
dribble_BLK220:
	ctx->vsp=local; return(local[0]);}

/*:plot*/
static pointer dribble_M228handtrajectorycontroller_plot(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<2) maerror();
	ctx->vsp=local+0;
	n=parsekeyparams(fqv[123], &argv[2], n-2, local+0, 0);
	if (n & (1<<0)) goto dribble_KEY230;
	local[0] = makeint((eusinteger_t)21L);
dribble_KEY230:
	local[1]= T;
	local[2]= fqv[124];
	ctx->vsp=local+3;
	w=(pointer)XFORMAT(ctx,2,local+1); /*format*/
	ctx->vsp=local+1;
	local[1]= makeclosure(codevec,quotevec,dribble_CLO231,env,argv,local);
	local[2]= argv[0];
	local[3]= fqv[115];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)MAPCAR(ctx,2,local+1); /*mapcar*/
	local[1]= w;
	w=local[1];
	if (!iscons(w) && w!=NIL) error(E_NOLIST);
	local[2]= (w)->c.cons.car;
	ctx->vsp=local+3;
	local[3]= makeclosure(codevec,quotevec,dribble_CLO232,env,argv,local);
	local[4]= loadglobal(fqv[121]);
	ctx->vsp=local+5;
	w=(pointer)MAPCAR(ctx,2,local+3); /*mapcar*/
	local[3]= w;
	w=local[3];
	if (!iscons(w) && w!=NIL) error(E_NOLIST);
	local[4]= (w)->c.cons.car;
	ctx->vsp=local+5;
	local[5]= makeclosure(codevec,quotevec,dribble_CLO233,env,argv,local);
	local[6]= argv[0];
	local[7]= fqv[116];
	ctx->vsp=local+8;
	w=(pointer)SEND(ctx,2,local+6); /*send*/
	local[6]= w;
	ctx->vsp=local+7;
	w=(pointer)MAPCAR(ctx,2,local+5); /*mapcar*/
	local[5]= w;
	ctx->vsp=local+6;
	local[6]= makeclosure(codevec,quotevec,dribble_CLO234,env,argv,local);
	local[7]= argv[0];
	local[8]= fqv[116];
	ctx->vsp=local+9;
	w=(pointer)SEND(ctx,2,local+7); /*send*/
	local[7]= w;
	ctx->vsp=local+8;
	w=(pointer)MAPCAR(ctx,2,local+6); /*mapcar*/
	local[6]= w;
	ctx->vsp=local+7;
	local[7]= makeclosure(codevec,quotevec,dribble_CLO235,env,argv,local);
	local[8]= argv[0];
	local[9]= fqv[116];
	ctx->vsp=local+10;
	w=(pointer)SEND(ctx,2,local+8); /*send*/
	local[8]= w;
	ctx->vsp=local+9;
	w=(pointer)MAPCAR(ctx,2,local+7); /*mapcar*/
	local[7]= w;
	ctx->vsp=local+8;
	local[8]= makeclosure(codevec,quotevec,dribble_CLO236,env,argv,local);
	local[9]= loadglobal(fqv[122]);
	ctx->vsp=local+10;
	w=(pointer)MAPCAR(ctx,2,local+8); /*mapcar*/
	local[8]= w;
	ctx->vsp=local+9;
	local[9]= makeclosure(codevec,quotevec,dribble_CLO237,env,argv,local);
	local[10]= loadglobal(fqv[122]);
	ctx->vsp=local+11;
	w=(pointer)MAPCAR(ctx,2,local+9); /*mapcar*/
	local[9]= w;
	ctx->vsp=local+10;
	local[10]= makeclosure(codevec,quotevec,dribble_CLO238,env,argv,local);
	local[11]= loadglobal(fqv[122]);
	ctx->vsp=local+12;
	w=(pointer)MAPCAR(ctx,2,local+10); /*mapcar*/
	local[10]= w;
	local[11]= local[0];
	local[12]= argv[0];
	local[13]= fqv[107];
	ctx->vsp=local+14;
	w=(pointer)SEND(ctx,2,local+12); /*send*/
	if (!iscons(w) && w!=NIL) error(E_NOLIST);
	local[12]= (w)->c.cons.car;
	local[13]= local[0];
	ctx->vsp=local+14;
	w=(pointer)ELT(ctx,2,local+12); /*elt*/
	local[12]= w;
	local[13]= NIL;
	local[14]= NIL;
	local[15]= fqv[125];
	ctx->vsp=local+16;
	w=(pointer)BOUNDP(ctx,1,local+15); /*boundp*/
	if (w!=NIL) goto dribble_IF239;
	ctx->vsp=local+15;
	w=(*ftab[11])(ctx,0,local+15,&ftab[11],fqv[126]); /*gnuplot*/
	local[15]= w;
	storeglobal(fqv[125],w);
	goto dribble_IF240;
dribble_IF239:
	local[15]= NIL;
dribble_IF240:
	local[15]= loadglobal(fqv[125]);
	local[16]= fqv[127];
	local[17]= local[1];
	local[18]= local[3];
	ctx->vsp=local+19;
	w=(pointer)SEND(ctx,4,local+15); /*send*/
	local[15]= fqv[128];
	ctx->vsp=local+16;
	w=(pointer)BOUNDP(ctx,1,local+15); /*boundp*/
	if (w!=NIL) goto dribble_IF241;
	ctx->vsp=local+15;
	w=(*ftab[11])(ctx,0,local+15,&ftab[11],fqv[126]); /*gnuplot*/
	local[15]= w;
	storeglobal(fqv[128],w);
	goto dribble_IF242;
dribble_IF241:
	local[15]= NIL;
dribble_IF242:
	local[15]= loadglobal(fqv[128]);
	local[16]= fqv[127];
	ctx->vsp=local+17;
	local[17]= makeclosure(codevec,quotevec,dribble_CLO243,env,argv,local);
	local[18]= local[1];
	ctx->vsp=local+19;
	w=(pointer)MAPCAR(ctx,2,local+17); /*mapcar*/
	local[17]= w;
	ctx->vsp=local+18;
	local[18]= makeclosure(codevec,quotevec,dribble_CLO244,env,argv,local);
	local[19]= local[3];
	ctx->vsp=local+20;
	w=(pointer)MAPCAR(ctx,2,local+18); /*mapcar*/
	local[18]= w;
	ctx->vsp=local+19;
	w=(pointer)SEND(ctx,4,local+15); /*send*/
	local[15]= T;
	local[16]= fqv[129];
	ctx->vsp=local+17;
	local[17]= makeclosure(codevec,quotevec,dribble_CLO245,env,argv,local);
	local[18]= local[1];
	ctx->vsp=local+19;
	w=(pointer)MAPCAR(ctx,2,local+17); /*mapcar*/
	local[17]= w;
	local[18]= makeint((eusinteger_t)0L);
	ctx->vsp=local+19;
	w=(pointer)ELT(ctx,2,local+17); /*elt*/
	local[17]= w;
	ctx->vsp=local+18;
	w=(pointer)XFORMAT(ctx,3,local+15); /*format*/
	local[15]= fqv[130];
	ctx->vsp=local+16;
	w=(pointer)BOUNDP(ctx,1,local+15); /*boundp*/
	if (w!=NIL) goto dribble_IF246;
	ctx->vsp=local+15;
	w=(*ftab[11])(ctx,0,local+15,&ftab[11],fqv[126]); /*gnuplot*/
	local[15]= w;
	storeglobal(fqv[130],w);
	goto dribble_IF247;
dribble_IF246:
	local[15]= NIL;
dribble_IF247:
	local[15]= loadglobal(fqv[130]);
	local[16]= fqv[127];
	local[17]= local[5];
	local[18]= local[6];
	local[19]= local[7];
	local[20]= local[8];
	local[21]= local[9];
	local[22]= local[10];
	ctx->vsp=local+23;
	w=(pointer)SEND(ctx,8,local+15); /*send*/
	local[15]= T;
	local[16]= fqv[131];
	local[17]= local[12];
	ctx->vsp=local+18;
	w=(pointer)XFORMAT(ctx,3,local+15); /*format*/
	local[15]= fqv[132];
	ctx->vsp=local+16;
	w=(pointer)BOUNDP(ctx,1,local+15); /*boundp*/
	if (w!=NIL) goto dribble_IF248;
	ctx->vsp=local+15;
	w=(*ftab[11])(ctx,0,local+15,&ftab[11],fqv[126]); /*gnuplot*/
	local[15]= w;
	storeglobal(fqv[132],w);
	goto dribble_IF249;
dribble_IF248:
	local[15]= NIL;
dribble_IF249:
	local[15]= fqv[133];
	ctx->vsp=local+16;
	w=(pointer)BOUNDP(ctx,1,local+15); /*boundp*/
	if (w!=NIL) goto dribble_IF250;
	ctx->vsp=local+15;
	w=(*ftab[11])(ctx,0,local+15,&ftab[11],fqv[126]); /*gnuplot*/
	local[15]= w;
	storeglobal(fqv[133],w);
	goto dribble_IF251;
dribble_IF250:
	local[15]= NIL;
dribble_IF251:
	ctx->vsp=local+15;
	local[15]= makeclosure(codevec,quotevec,dribble_CLO252,env,argv,local);
	local[16]= argv[0];
	local[17]= fqv[107];
	ctx->vsp=local+18;
	w=(pointer)SEND(ctx,2,local+16); /*send*/
	local[16]= w;
	ctx->vsp=local+17;
	w=(pointer)MAPCAR(ctx,2,local+15); /*mapcar*/
	local[13] = w;
	ctx->vsp=local+15;
	local[15]= makeclosure(codevec,quotevec,dribble_CLO253,env,argv,local);
	local[16]= argv[0];
	local[17]= fqv[107];
	ctx->vsp=local+18;
	w=(pointer)SEND(ctx,2,local+16); /*send*/
	local[16]= w;
	ctx->vsp=local+17;
	w=(pointer)MAPCAR(ctx,2,local+15); /*mapcar*/
	local[14] = w;
	local[15]= loadglobal(fqv[133]);
	local[16]= fqv[127];
	local[17]= local[13];
	ctx->vsp=local+18;
	w=(pointer)SEND(ctx,3,local+15); /*send*/
	local[15]= loadglobal(fqv[132]);
	local[16]= fqv[127];
	local[17]= local[14];
	ctx->vsp=local+18;
	w=(pointer)SEND(ctx,3,local+15); /*send*/
	local[15]= T;
	local[16]= fqv[134];
	local[17]= local[13];
	local[18]= makeint((eusinteger_t)0L);
	ctx->vsp=local+19;
	w=(pointer)ELT(ctx,2,local+17); /*elt*/
	local[17]= w;
	ctx->vsp=local+18;
	w=(pointer)XFORMAT(ctx,3,local+15); /*format*/
	local[15]= T;
	local[16]= fqv[135];
	local[17]= local[13];
	local[18]= makeint((eusinteger_t)1L);
	ctx->vsp=local+19;
	w=(pointer)ELT(ctx,2,local+17); /*elt*/
	local[17]= w;
	ctx->vsp=local+18;
	w=(pointer)XFORMAT(ctx,3,local+15); /*format*/
	local[15]= T;
	local[16]= fqv[136];
	local[17]= local[14];
	local[18]= makeint((eusinteger_t)0L);
	ctx->vsp=local+19;
	w=(pointer)ELT(ctx,2,local+17); /*elt*/
	local[17]= w;
	ctx->vsp=local+18;
	w=(pointer)XFORMAT(ctx,3,local+15); /*format*/
	local[15]= T;
	local[16]= fqv[137];
	local[17]= local[14];
	local[18]= makeint((eusinteger_t)1L);
	ctx->vsp=local+19;
	w=(pointer)ELT(ctx,2,local+17); /*elt*/
	local[17]= w;
	ctx->vsp=local+18;
	w=(pointer)XFORMAT(ctx,3,local+15); /*format*/
	local[1]= T;
	local[2]= fqv[138];
	ctx->vsp=local+3;
	w=(pointer)XFORMAT(ctx,2,local+1); /*format*/
	local[0]= w;
dribble_BLK229:
	ctx->vsp=local; return(local[0]);}

/*:dribble-mode2*/
static pointer dribble_M254handtrajectorycontroller_dribble_mode2(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=2) maerror();
	local[0]= argv[0];
	local[1]= fqv[139];
	local[2]= fqv[35];
	local[3]= T;
	local[4]= fqv[92];
	local[5]= makeint((eusinteger_t)5000L);
	ctx->vsp=local+6;
	w=(pointer)SEND(ctx,6,local+0); /*send*/
	local[0]= loadglobal(fqv[37]);
	local[1]= fqv[46];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= makeint((eusinteger_t)1000000L);
	ctx->vsp=local+1;
	w=(*ftab[12])(ctx,1,local+0,&ftab[12],fqv[140]); /*unix:usleep*/
	local[0]= argv[0];
	local[1]= fqv[91];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= T;
	local[1]= fqv[141];
	ctx->vsp=local+2;
	w=(pointer)XFORMAT(ctx,2,local+0); /*format*/
	local[0]= T;
	local[1]= fqv[142];
	local[2]= loadglobal(fqv[21]);
	local[3]= fqv[60];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)XFORMAT(ctx,3,local+0); /*format*/
	local[0]= T;
	local[1]= fqv[143];
	local[2]= loadglobal(fqv[21]);
	local[3]= fqv[53];
	local[4]= fqv[41];
	local[5]= fqv[42];
	ctx->vsp=local+6;
	w=(pointer)SEND(ctx,4,local+2); /*send*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)XFORMAT(ctx,3,local+0); /*format*/
	local[0]= loadglobal(fqv[21]);
	local[1]= fqv[144];
	local[2]= fqv[60];
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,3,local+0); /*send*/
	local[0]= w;
	storeglobal(fqv[145],w);
	local[0]= loadglobal(fqv[21]);
	local[1]= fqv[60];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= w;
	storeglobal(fqv[146],w);
	local[0]= T;
	local[1]= fqv[147];
	local[2]= loadglobal(fqv[21]);
	local[3]= fqv[144];
	local[4]= fqv[60];
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,3,local+2); /*send*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)XFORMAT(ctx,3,local+0); /*format*/
	local[0]= T;
	local[1]= fqv[148];
	local[2]= argv[0];
	local[3]= fqv[62];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)XFORMAT(ctx,3,local+0); /*format*/
	local[0]= T;
	local[1]= fqv[149];
	local[2]= argv[0];
	local[3]= fqv[64];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)XFORMAT(ctx,3,local+0); /*format*/
	local[0]= T;
	local[1]= fqv[150];
	ctx->vsp=local+2;
	w=(pointer)XFORMAT(ctx,2,local+0); /*format*/
	local[0]= argv[0];
	local[1]= fqv[118];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= argv[0];
	local[1]= fqv[120];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= argv[0];
	local[1]= fqv[151];
	local[2]= fqv[88];
	local[3]= T;
	local[4]= fqv[89];
	local[5]= T;
	ctx->vsp=local+6;
	w=(pointer)SEND(ctx,6,local+0); /*send*/
	local[0]= T;
	local[1]= fqv[152];
	local[2]= argv[0];
	local[3]= fqv[107];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)LENGTH(ctx,1,local+2); /*length*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)XFORMAT(ctx,3,local+0); /*format*/
	ctx->vsp=local+0;
	w=(*ftab[13])(ctx,0,local+0,&ftab[13],fqv[153]); /*ri2robot*/
	local[0]= T;
	local[1]= fqv[154];
	ctx->vsp=local+2;
	w=(pointer)XFORMAT(ctx,2,local+0); /*format*/
	local[0]= T;
	local[1]= fqv[155];
	local[2]= argv[0];
	local[3]= fqv[107];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	local[3]= makeint((eusinteger_t)0L);
	ctx->vsp=local+4;
	w=(pointer)ELT(ctx,2,local+2); /*elt*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)XFORMAT(ctx,3,local+0); /*format*/
	local[0]= T;
	local[1]= fqv[156];
	local[2]= argv[0];
	local[3]= fqv[117];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	local[3]= makeint((eusinteger_t)0L);
	ctx->vsp=local+4;
	w=(pointer)ELT(ctx,2,local+2); /*elt*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)XFORMAT(ctx,3,local+0); /*format*/
	local[0]= loadglobal(fqv[21]);
	local[1]= fqv[60];
	local[2]= argv[0];
	local[3]= fqv[107];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	local[3]= makeint((eusinteger_t)0L);
	ctx->vsp=local+4;
	w=(pointer)ELT(ctx,2,local+2); /*elt*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,3,local+0); /*send*/
	local[0]= loadglobal(fqv[21]);
	local[1]= fqv[144];
	local[2]= fqv[60];
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,3,local+0); /*send*/
	local[0]= w;
	storeglobal(fqv[157],w);
	local[0]= loadglobal(fqv[21]);
	local[1]= fqv[60];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	argv[0]->c.obj.iv[15] = w;
	local[0]= T;
	local[1]= fqv[158];
	local[2]= loadglobal(fqv[21]);
	local[3]= fqv[144];
	local[4]= fqv[60];
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,3,local+2); /*send*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)XFORMAT(ctx,3,local+0); /*format*/
	local[0]= T;
	local[1]= fqv[159];
	local[2]= loadglobal(fqv[157]);
	local[3]= loadglobal(fqv[145]);
	ctx->vsp=local+4;
	w=(pointer)VMINUS(ctx,2,local+2); /*v-*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)XFORMAT(ctx,3,local+0); /*format*/
	local[0]= T;
	local[1]= fqv[160];
	local[2]= argv[0];
	local[3]= fqv[107];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	local[3]= makeint((eusinteger_t)0L);
	ctx->vsp=local+4;
	w=(pointer)ELT(ctx,2,local+2); /*elt*/
	local[2]= w;
	local[3]= loadglobal(fqv[146]);
	ctx->vsp=local+4;
	w=(pointer)VMINUS(ctx,2,local+2); /*v-*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)XFORMAT(ctx,3,local+0); /*format*/
	local[0]= T;
	local[1]= fqv[161];
	local[2]= argv[0];
	local[3]= fqv[107];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	local[3]= makeint((eusinteger_t)1L);
	ctx->vsp=local+4;
	w=(pointer)ELT(ctx,2,local+2); /*elt*/
	local[2]= w;
	local[3]= argv[0];
	local[4]= fqv[107];
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,2,local+3); /*send*/
	local[3]= w;
	local[4]= makeint((eusinteger_t)0L);
	ctx->vsp=local+5;
	w=(pointer)ELT(ctx,2,local+3); /*elt*/
	local[3]= w;
	ctx->vsp=local+4;
	w=(pointer)VMINUS(ctx,2,local+2); /*v-*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)XFORMAT(ctx,3,local+0); /*format*/
	local[0]= T;
	local[1]= fqv[162];
	local[2]= argv[0];
	local[3]= fqv[62];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)XFORMAT(ctx,3,local+0); /*format*/
	local[0]= T;
	local[1]= fqv[163];
	local[2]= argv[0];
	local[3]= fqv[64];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)XFORMAT(ctx,3,local+0); /*format*/
	local[0]= T;
	local[1]= fqv[164];
	ctx->vsp=local+2;
	w=(pointer)XFORMAT(ctx,2,local+0); /*format*/
	local[0]= argv[0];
	local[1]= fqv[165];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= T;
	local[1]= fqv[166];
	ctx->vsp=local+2;
	w=(pointer)XFORMAT(ctx,2,local+0); /*format*/
	local[0]= w;
dribble_BLK255:
	ctx->vsp=local; return(local[0]);}

/*closure or cleaner*/
static pointer dribble_CLO231(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=1) maerror();
	local[0]= argv[0];
	local[1]= makeint((eusinteger_t)2L);
	ctx->vsp=local+2;
	w=(pointer)ELT(ctx,2,local+0); /*elt*/
	local[0]= w;
	ctx->vsp=local; return(local[0]);}

/*closure or cleaner*/
static pointer dribble_CLO232(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=1) maerror();
	local[0]= argv[0];
	local[1]= makeint((eusinteger_t)2L);
	ctx->vsp=local+2;
	w=(pointer)ELT(ctx,2,local+0); /*elt*/
	local[0]= w;
	ctx->vsp=local; return(local[0]);}

/*closure or cleaner*/
static pointer dribble_CLO233(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=1) maerror();
	local[0]= argv[0];
	local[1]= makeint((eusinteger_t)0L);
	ctx->vsp=local+2;
	w=(pointer)ELT(ctx,2,local+0); /*elt*/
	local[0]= w;
	ctx->vsp=local; return(local[0]);}

/*closure or cleaner*/
static pointer dribble_CLO234(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=1) maerror();
	local[0]= argv[0];
	local[1]= makeint((eusinteger_t)1L);
	ctx->vsp=local+2;
	w=(pointer)ELT(ctx,2,local+0); /*elt*/
	local[0]= w;
	ctx->vsp=local; return(local[0]);}

/*closure or cleaner*/
static pointer dribble_CLO235(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=1) maerror();
	local[0]= argv[0];
	local[1]= makeint((eusinteger_t)2L);
	ctx->vsp=local+2;
	w=(pointer)ELT(ctx,2,local+0); /*elt*/
	local[0]= w;
	ctx->vsp=local; return(local[0]);}

/*closure or cleaner*/
static pointer dribble_CLO236(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=1) maerror();
	local[0]= argv[0];
	local[1]= makeint((eusinteger_t)0L);
	ctx->vsp=local+2;
	w=(pointer)ELT(ctx,2,local+0); /*elt*/
	local[0]= w;
	ctx->vsp=local; return(local[0]);}

/*closure or cleaner*/
static pointer dribble_CLO237(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=1) maerror();
	local[0]= argv[0];
	local[1]= makeint((eusinteger_t)1L);
	ctx->vsp=local+2;
	w=(pointer)ELT(ctx,2,local+0); /*elt*/
	local[0]= w;
	ctx->vsp=local; return(local[0]);}

/*closure or cleaner*/
static pointer dribble_CLO238(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=1) maerror();
	local[0]= argv[0];
	local[1]= makeint((eusinteger_t)2L);
	ctx->vsp=local+2;
	w=(pointer)ELT(ctx,2,local+0); /*elt*/
	local[0]= w;
	ctx->vsp=local; return(local[0]);}

/*closure or cleaner*/
static pointer dribble_CLO243(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=1) maerror();
	local[0]= NIL;
	local[1]= argv[0];
	local[2]= env->c.clo.env2[2];
	ctx->vsp=local+3;
	w=(pointer)MINUS(ctx,2,local+1); /*-*/
	local[0] = w;
	env->c.clo.env2[2] = argv[0];
	w = local[0];
	local[0]= w;
	ctx->vsp=local; return(local[0]);}

/*closure or cleaner*/
static pointer dribble_CLO244(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=1) maerror();
	local[0]= NIL;
	local[1]= argv[0];
	local[2]= env->c.clo.env2[4];
	ctx->vsp=local+3;
	w=(pointer)MINUS(ctx,2,local+1); /*-*/
	local[0] = w;
	env->c.clo.env2[4] = argv[0];
	w = local[0];
	local[0]= w;
	ctx->vsp=local; return(local[0]);}

/*closure or cleaner*/
static pointer dribble_CLO245(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=1) maerror();
	local[0]= NIL;
	local[1]= argv[0];
	local[2]= env->c.clo.env2[2];
	ctx->vsp=local+3;
	w=(pointer)MINUS(ctx,2,local+1); /*-*/
	local[0] = w;
	env->c.clo.env2[2] = argv[0];
	w = local[0];
	local[0]= w;
	ctx->vsp=local; return(local[0]);}

/*closure or cleaner*/
static pointer dribble_CLO252(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=1) maerror();
	local[0]= argv[0];
	local[1]= env->c.clo.env2[0];
	ctx->vsp=local+2;
	w=(pointer)ELT(ctx,2,local+0); /*elt*/
	local[0]= w;
	ctx->vsp=local; return(local[0]);}

/*closure or cleaner*/
static pointer dribble_CLO253(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=1) maerror();
	local[0]= argv[0];
	local[1]= env->c.clo.env2[0];
	ctx->vsp=local+2;
	w=(pointer)ELT(ctx,2,local+0); /*elt*/
	local[0]= w;
	local[1]= NIL;
	local[2]= local[0];
	local[3]= env->c.clo.env2[12];
	ctx->vsp=local+4;
	w=(pointer)MINUS(ctx,2,local+2); /*-*/
	local[1] = w;
	env->c.clo.env2[12] = local[0];
	w = local[1];
	local[0]= w;
	ctx->vsp=local; return(local[0]);}

/*start-ab*/
static pointer dribble_F2start_ab(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=0) maerror();
	local[0]= loadglobal(fqv[37]);
	local[1]= fqv[167];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= w;
dribble_BLK256:
	ctx->vsp=local; return(local[0]);}

/*stop-ab*/
static pointer dribble_F3stop_ab(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=0) maerror();
	local[0]= loadglobal(fqv[37]);
	local[1]= fqv[168];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= w;
dribble_BLK257:
	ctx->vsp=local; return(local[0]);}

/*start-st*/
static pointer dribble_F4start_st(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=0) maerror();
	local[0]= loadglobal(fqv[37]);
	local[1]= fqv[169];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= w;
dribble_BLK258:
	ctx->vsp=local; return(local[0]);}

/*stop-st*/
static pointer dribble_F5stop_st(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=0) maerror();
	local[0]= loadglobal(fqv[37]);
	local[1]= fqv[170];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= w;
dribble_BLK259:
	ctx->vsp=local; return(local[0]);}

/*start-ee*/
static pointer dribble_F6start_ee(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=0) maerror();
	local[0]= loadglobal(fqv[37]);
	local[1]= fqv[171];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= w;
dribble_BLK260:
	ctx->vsp=local; return(local[0]);}

/*stop-ee*/
static pointer dribble_F7stop_ee(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=0) maerror();
	local[0]= loadglobal(fqv[37]);
	local[1]= fqv[172];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= w;
dribble_BLK261:
	ctx->vsp=local; return(local[0]);}

/*start-d-mode*/
static pointer dribble_F8start_d_mode(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=0) maerror();
	local[0]= loadglobal(fqv[37]);
	local[1]= fqv[173];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= w;
dribble_BLK262:
	ctx->vsp=local; return(local[0]);}

/*stop-d-mode*/
static pointer dribble_F9stop_d_mode(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=0) maerror();
	local[0]= loadglobal(fqv[37]);
	local[1]= fqv[174];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= w;
dribble_BLK263:
	ctx->vsp=local; return(local[0]);}

/*start-d-motion*/
static pointer dribble_F10start_d_motion(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=0) maerror();
	local[0]= loadglobal(fqv[37]);
	local[1]= fqv[175];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= w;
dribble_BLK264:
	ctx->vsp=local; return(local[0]);}

/*stop-d-motion*/
static pointer dribble_F11stop_d_motion(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=0) maerror();
	local[0]= loadglobal(fqv[37]);
	local[1]= fqv[176];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= w;
dribble_BLK265:
	ctx->vsp=local; return(local[0]);}

/*start-dribble-mode*/
static pointer dribble_F12start_dribble_mode(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=0) maerror();
	ctx->vsp=local+0;
	w=(pointer)dribble_F2start_ab(ctx,0,local+0); /*start-ab*/
	ctx->vsp=local+0;
	w=(pointer)dribble_F4start_st(ctx,0,local+0); /*start-st*/
	local[0]= makeint((eusinteger_t)1000000L);
	ctx->vsp=local+1;
	w=(*ftab[12])(ctx,1,local+0,&ftab[12],fqv[140]); /*unix:usleep*/
	ctx->vsp=local+0;
	w=(pointer)dribble_F8start_d_mode(ctx,0,local+0); /*start-d-mode*/
	local[0]= makeint((eusinteger_t)1000000L);
	ctx->vsp=local+1;
	w=(*ftab[12])(ctx,1,local+0,&ftab[12],fqv[140]); /*unix:usleep*/
	ctx->vsp=local+0;
	w=(pointer)dribble_F6start_ee(ctx,0,local+0); /*start-ee*/
	local[0]= w;
dribble_BLK266:
	ctx->vsp=local; return(local[0]);}

/*stop-dribble-mode*/
static pointer dribble_F13stop_dribble_mode(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n!=0) maerror();
	ctx->vsp=local+0;
	w=(pointer)dribble_F9stop_d_mode(ctx,0,local+0); /*stop-d-mode*/
	ctx->vsp=local+0;
	w=(pointer)dribble_F7stop_ee(ctx,0,local+0); /*stop-ee*/
	local[0]= w;
dribble_BLK267:
	ctx->vsp=local; return(local[0]);}

/*move*/
static pointer dribble_F14move(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<0) maerror();
	ctx->vsp=local+0;
	n=parsekeyparams(fqv[177], &argv[0], n-0, local+0, 0);
	if (n & (1<<0)) goto dribble_KEY269;
	local[2]= makeint((eusinteger_t)5000L);
	goto dribble_KEY270;
dribble_KEY269:
	local[2]= local[0];
dribble_KEY270:
	w = local[2];
	ctx->vsp=local+2;
	bindspecial(ctx,fqv[19],w);
	if (n & (1<<1)) goto dribble_KEY271;
	local[1] = T;
dribble_KEY271:
	local[5]= loadglobal(fqv[37]);
	local[6]= fqv[60];
	local[7]= loadglobal(fqv[21]);
	local[8]= fqv[60];
	ctx->vsp=local+9;
	w=(pointer)SEND(ctx,2,local+7); /*send*/
	local[7]= w;
	local[8]= loadglobal(fqv[19]);
	local[9]= NIL;
	local[10]= makeint((eusinteger_t)0L);
	local[11]= fqv[178];
	local[12]= loadglobal(fqv[19]);
	local[13]= makeint((eusinteger_t)1000L);
	ctx->vsp=local+14;
	w=(pointer)QUOTIENT(ctx,2,local+12); /*/*/
	local[12]= w;
	ctx->vsp=local+13;
	w=(pointer)SEND(ctx,8,local+5); /*send*/
	if (local[1]==NIL) goto dribble_IF272;
	local[5]= loadglobal(fqv[37]);
	local[6]= fqv[46];
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,2,local+5); /*send*/
	local[5]= w;
	goto dribble_IF273;
dribble_IF272:
	local[5]= NIL;
dribble_IF273:
	local[5]= loadglobal(fqv[21]);
	local[6]= fqv[60];
	ctx->vsp=local+7;
	w=(pointer)SEND(ctx,2,local+5); /*send*/
	local[5]= w;
	ctx->vsp=local+6;
	unbindx(ctx,1);
	w = local[5];
	local[0]= w;
dribble_BLK268:
	ctx->vsp=local; return(local[0]);}

/*lower-waist*/
static pointer dribble_F15lower_waist(ctx,n,argv,env)
register context *ctx;
register int n; register pointer argv[]; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv=qv;
  numunion nu;
	if (n<0) maerror();
	ctx->vsp=local+0;
	n=parsekeyparams(fqv[179], &argv[0], n-0, local+0, 0);
	if (n & (1<<0)) goto dribble_KEY275;
	local[0] = makeint((eusinteger_t)120L);
dribble_KEY275:
	if (n & (1<<1)) goto dribble_KEY276;
	local[1] = NIL;
dribble_KEY276:
	local[2]= loadglobal(fqv[21]);
	local[3]= fqv[180];
	ctx->vsp=local+4;
	w=(*ftab[0])(ctx,0,local+4,&ftab[0],fqv[20]); /*make-coords*/
	local[4]= w;
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,3,local+2); /*send*/
	local[2]= loadglobal(fqv[21]);
	local[3]= fqv[181];
	local[4]= fqv[54];
	local[5]= makeint((eusinteger_t)0L);
	local[6]= makeint((eusinteger_t)0L);
	local[7]= local[0];
	ctx->vsp=local+8;
	w=(pointer)MKFLTVEC(ctx,3,local+5); /*float-vector*/
	local[5]= w;
	ctx->vsp=local+6;
	w=(pointer)SEND(ctx,4,local+2); /*send*/
	local[2]= loadglobal(fqv[21]);
	local[3]= fqv[182];
	local[4]= fqv[183];
	local[5]= fqv[184];
	ctx->vsp=local+6;
	w=(pointer)SEND(ctx,4,local+2); /*send*/
	if (local[1]==NIL) goto dribble_IF277;
	local[2]= loadglobal(fqv[37]);
	local[3]= fqv[60];
	local[4]= loadglobal(fqv[21]);
	local[5]= fqv[60];
	ctx->vsp=local+6;
	w=(pointer)SEND(ctx,2,local+4); /*send*/
	local[4]= w;
	local[5]= makeint((eusinteger_t)10000L);
	ctx->vsp=local+6;
	w=(pointer)SEND(ctx,4,local+2); /*send*/
	local[2]= loadglobal(fqv[37]);
	local[3]= fqv[46];
	ctx->vsp=local+4;
	w=(pointer)SEND(ctx,2,local+2); /*send*/
	local[2]= w;
	goto dribble_IF278;
dribble_IF277:
	local[2]= NIL;
dribble_IF278:
	local[2]= loadglobal(fqv[21]);
	local[3]= fqv[185];
	ctx->vsp=local+4;
	w=(*ftab[0])(ctx,0,local+4,&ftab[0],fqv[20]); /*make-coords*/
	local[4]= w;
	ctx->vsp=local+5;
	w=(pointer)SEND(ctx,3,local+2); /*send*/
	local[0]= w;
dribble_BLK274:
	ctx->vsp=local; return(local[0]);}

/* initializer*/
pointer ___dribble_motion(ctx,n,argv,env)
register context *ctx; int n; pointer *argv; pointer env;
{ register pointer *local=ctx->vsp, w, *fqv;
  register int i;
  numunion nu;
  module=argv[0];
  quotevec=build_quote_vector(ctx,QUOTE_STRINGS_SIZE, quote_strings);
  module->c.code.quotevec=quotevec;
  codevec=module->c.code.codevec;
  fqv=qv=quotevec->c.vec.v;
  init_ftab();
	ctx->vsp=local+0;
	compfun(ctx,fqv[186],module,dribble_F1__gc_print,fqv[187]);
	local[0]= fqv[186];
	storeglobal(fqv[188],local[0]);
	ctx->vsp=local+0;
	compmacro(ctx,fqv[189],module,dribble_F17,fqv[190]);
	local[0]= fqv[191];
	ctx->vsp=local+1;
	w=(pointer)BOUNDP(ctx,1,local+0); /*boundp*/
	if (w!=NIL) goto dribble_IF279;
	ctx->vsp=local+0;
	w=(*ftab[14])(ctx,0,local+0,&ftab[14],fqv[192]); /*jaxon_red-init*/
	local[0]= loadglobal(fqv[191]);
	storeglobal(fqv[21],local[0]);
	goto dribble_IF280;
dribble_IF279:
	local[0]= NIL;
dribble_IF280:
	local[0]= loadglobal(fqv[21]);
	ctx->vsp=local+1;
	w=(pointer)LIST(ctx,1,local+0); /*list*/
	local[0]= w;
	ctx->vsp=local+1;
	w=(*ftab[15])(ctx,1,local+0,&ftab[15],fqv[193]); /*objects*/
	local[0]= loadglobal(fqv[194]);
	local[1]= fqv[195];
	local[2]= fqv[196];
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,3,local+0); /*send*/
	local[0]= loadglobal(fqv[194]);
	local[1]= fqv[197];
	ctx->vsp=local+2;
	w=(pointer)SEND(ctx,2,local+0); /*send*/
	local[0]= loadglobal(fqv[37]);
	local[1]= fqv[198];
	local[2]= makeint((eusinteger_t)500L);
	{ eusinteger_t i,j;
		j=intval(makeint((eusinteger_t)60L)); i=intval(local[2]);
		local[2]=(makeint(i * j));}
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,3,local+0); /*send*/
	local[0]= T;
	local[1]= fqv[199];
	ctx->vsp=local+2;
	w=(pointer)XFORMAT(ctx,2,local+0); /*format*/
	local[0]= T;
	local[1]= fqv[200];
	ctx->vsp=local+2;
	w=(pointer)XFORMAT(ctx,2,local+0); /*format*/
	local[0]= T;
	local[1]= fqv[201];
	ctx->vsp=local+2;
	w=(pointer)XFORMAT(ctx,2,local+0); /*format*/
	local[0]= T;
	local[1]= fqv[202];
	ctx->vsp=local+2;
	w=(pointer)XFORMAT(ctx,2,local+0); /*format*/
	local[0]= T;
	local[1]= fqv[203];
	ctx->vsp=local+2;
	w=(pointer)XFORMAT(ctx,2,local+0); /*format*/
	local[0]= T;
	local[1]= fqv[204];
	ctx->vsp=local+2;
	w=(pointer)XFORMAT(ctx,2,local+0); /*format*/
	local[0]= loadglobal(fqv[21]);
	local[1]= fqv[205];
	local[2]= fqv[43];
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,3,local+0); /*send*/
	local[0]= w;
	storeglobal(fqv[94],w);
	local[0]= fqv[206];
	ctx->vsp=local+1;
	w=(*ftab[16])(ctx,1,local+0,&ftab[16],fqv[207]); /*ros::roseus-add-msgs*/
	local[0]= fqv[208];
	ctx->vsp=local+1;
	w=(*ftab[16])(ctx,1,local+0,&ftab[16],fqv[207]); /*ros::roseus-add-msgs*/
	local[0]= NIL;
	storeglobal(fqv[16],local[0]);
	local[0]= NIL;
	storeglobal(fqv[17],local[0]);
	local[0]= loadglobal(fqv[29]);
	ctx->vsp=local+1;
	w=(pointer)INSTANTIATE(ctx,1,local+0); /*instantiate*/
	local[0]= w;
	local[1]= local[0];
	local[2]= fqv[93];
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,2,local+1); /*send*/
	w = local[0];
	local[0]= w;
	storeglobal(fqv[18],w);
	local[0]= loadglobal(fqv[18]);
	local[1]= fqv[43];
	local[2]= loadglobal(fqv[94]);
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,3,local+0); /*send*/
	local[0]= NIL;
	storeglobal(fqv[121],local[0]);
	local[0]= NIL;
	storeglobal(fqv[122],local[0]);
	local[0]= NIL;
	storeglobal(fqv[209],local[0]);
	local[0]= fqv[210];
	local[1]= fqv[211];
	local[2]= fqv[210];
	local[3]= fqv[212];
	local[4]= loadglobal(fqv[213]);
	local[5]= fqv[214];
	local[6]= fqv[215];
	local[7]= fqv[216];
	local[8]= NIL;
	local[9]= fqv[217];
	local[10]= NIL;
	local[11]= fqv[218];
	local[12]= makeint((eusinteger_t)-1L);
	local[13]= fqv[219];
	local[14]= NIL;
	ctx->vsp=local+15;
	w=(*ftab[17])(ctx,13,local+2,&ftab[17],fqv[220]); /*make-class*/
	local[2]= w;
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,3,local+0); /*send*/
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M20handtrajectorycontroller_init,fqv[93],fqv[210],fqv[221]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M22handtrajectorycontroller_object_state,fqv[222],fqv[210],fqv[223]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M28handtrajectorycontroller_tactile_state,fqv[224],fqv[210],fqv[225]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M34handtrajectorycontroller_joint_states,fqv[226],fqv[210],fqv[227]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M40handtrajectorycontroller_ref_pose,fqv[109],fqv[210],fqv[228]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M46handtrajectorycontroller_target_z_start,fqv[62],fqv[210],fqv[229]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M52handtrajectorycontroller_target_z_goal,fqv[67],fqv[210],fqv[230]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M58handtrajectorycontroller_target_pitch_start,fqv[64],fqv[210],fqv[231]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M64handtrajectorycontroller_target_pitch_goal,fqv[68],fqv[210],fqv[232]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M70handtrajectorycontroller_motion_period,fqv[69],fqv[210],fqv[233]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M76handtrajectorycontroller_scale_period,fqv[70],fqv[210],fqv[234]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M82handtrajectorycontroller_dt,fqv[235],fqv[210],fqv[236]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M88handtrajectorycontroller_time,fqv[92],fqv[210],fqv[237]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M94handtrajectorycontroller_dt_list,fqv[110],fqv[210],fqv[238]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M100handtrajectorycontroller_target_hand_pos,fqv[77],fqv[210],fqv[239]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M106handtrajectorycontroller_target_hand_rpy,fqv[79],fqv[210],fqv[240]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M112handtrajectorycontroller_target_hand_coords,fqv[75],fqv[210],fqv[241]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M118handtrajectorycontroller_target_angle_vector,fqv[242],fqv[210],fqv[243]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M124handtrajectorycontroller_target_hand_pos_list,fqv[115],fqv[210],fqv[244]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M130handtrajectorycontroller_target_hand_rpy_list,fqv[116],fqv[210],fqv[245]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M136handtrajectorycontroller_target_hand_coords_list,fqv[117],fqv[210],fqv[246]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M142handtrajectorycontroller_target_angle_vector_list,fqv[107],fqv[210],fqv[247]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M148handtrajectorycontroller_ros_init,fqv[248],fqv[210],fqv[249]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M150handtrajectorycontroller_object_state_cb,fqv[250],fqv[210],fqv[251]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M154handtrajectorycontroller_dribble_pose,fqv[139],fqv[210],fqv[252]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M159handtrajectorycontroller_update_param,fqv[91],fqv[210],fqv[253]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M165handtrajectorycontroller_make_target_hand_coords,fqv[83],fqv[210],fqv[254]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M168handtrajectorycontroller_make_target_hand_pos,fqv[72],fqv[210],fqv[255]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M171handtrajectorycontroller_make_target_hand_rpy,fqv[74],fqv[210],fqv[256]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M174handtrajectorycontroller_solve_ik,fqv[151],fqv[210],fqv[257]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M180handtrajectorycontroller_dribble_mode,fqv[258],fqv[210],fqv[259]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M183handtrajectorycontroller_motion_cb,fqv[103],fqv[210],fqv[260]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M185handtrajectorycontroller_start_motion,fqv[261],fqv[210],fqv[262]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M190handtrajectorycontroller_start_motion2,fqv[263],fqv[210],fqv[264]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M192handtrajectorycontroller_update_param2,fqv[265],fqv[210],fqv[266]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M198handtrajectorycontroller_make_dt_list,fqv[118],fqv[210],fqv[267]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M200handtrajectorycontroller_make_target_hand_coords_list,fqv[120],fqv[210],fqv[268]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M205handtrajectorycontroller_make_target_hand_pos_list,fqv[113],fqv[210],fqv[269]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M212handtrajectorycontroller_make_target_hand_rpy_list,fqv[114],fqv[210],fqv[270]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M219handtrajectorycontroller_solve_ik2,fqv[271],fqv[210],fqv[272]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M228handtrajectorycontroller_plot,fqv[165],fqv[210],fqv[273]);
	ctx->vsp=local+0;
	addcmethod(ctx,module,dribble_M254handtrajectorycontroller_dribble_mode2,fqv[274],fqv[210],fqv[275]);
	local[0]= loadglobal(fqv[210]);
	ctx->vsp=local+1;
	w=(pointer)INSTANTIATE(ctx,1,local+0); /*instantiate*/
	local[0]= w;
	local[1]= local[0];
	local[2]= fqv[93];
	ctx->vsp=local+3;
	w=(pointer)SEND(ctx,2,local+1); /*send*/
	w = local[0];
	local[0]= w;
	storeglobal(fqv[276],w);
	ctx->vsp=local+0;
	compfun(ctx,fqv[277],module,dribble_F2start_ab,fqv[278]);
	ctx->vsp=local+0;
	compfun(ctx,fqv[279],module,dribble_F3stop_ab,fqv[280]);
	ctx->vsp=local+0;
	compfun(ctx,fqv[281],module,dribble_F4start_st,fqv[282]);
	ctx->vsp=local+0;
	compfun(ctx,fqv[283],module,dribble_F5stop_st,fqv[284]);
	ctx->vsp=local+0;
	compfun(ctx,fqv[285],module,dribble_F6start_ee,fqv[286]);
	ctx->vsp=local+0;
	compfun(ctx,fqv[287],module,dribble_F7stop_ee,fqv[288]);
	ctx->vsp=local+0;
	compfun(ctx,fqv[289],module,dribble_F8start_d_mode,fqv[290]);
	ctx->vsp=local+0;
	compfun(ctx,fqv[291],module,dribble_F9stop_d_mode,fqv[292]);
	ctx->vsp=local+0;
	compfun(ctx,fqv[293],module,dribble_F10start_d_motion,fqv[294]);
	ctx->vsp=local+0;
	compfun(ctx,fqv[295],module,dribble_F11stop_d_motion,fqv[296]);
	ctx->vsp=local+0;
	compfun(ctx,fqv[297],module,dribble_F12start_dribble_mode,fqv[298]);
	ctx->vsp=local+0;
	compfun(ctx,fqv[299],module,dribble_F13stop_dribble_mode,fqv[300]);
	ctx->vsp=local+0;
	compfun(ctx,fqv[301],module,dribble_F14move,fqv[302]);
	ctx->vsp=local+0;
	compfun(ctx,fqv[303],module,dribble_F15lower_waist,fqv[304]);
	local[0]= NIL;
	ctx->vsp=local; return(local[0]);}
static void init_ftab()
{  register int i;
  for (i=0; i<18; i++) ftab[i]=fcallx;
}
