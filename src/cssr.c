/*------------------------------------------------------------------------------
* cssr.c : Compact SSR message decode functions
*
* references :
*     see rtcm3.c
*
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

/* constants -----------------------------------------------------------------*/

/* ssr update intervals ------------------------------------------------------*/
static const double ssrudint[16]={
	1,2,5,10,15,30,60,120,240,300,600,900,1800,3600,7200,10800
};

static const int sig_tbl[8][16] = {
  {	CODE_L1C,CODE_L1P,CODE_L1W,CODE_L1D,CODE_L1P,CODE_L1X,CODE_L2S,CODE_L2L,
	CODE_L2X,CODE_L2P,CODE_L2W,CODE_L5I,CODE_L5Q,CODE_L5X,       0,       0}, /* GPS */
  {	CODE_L1C,CODE_L1P,CODE_L2C,CODE_L2P,CODE_L4A,CODE_L4B,CODE_L4X,CODE_L6A,
	CODE_L6B,CODE_L6X,CODE_L3I,CODE_L3Q,CODE_L3X,       0,       0,       0}, /* GLO */
  {	CODE_L1B,CODE_L1C,CODE_L1X,CODE_L5I,CODE_L5Q,CODE_L5X,CODE_L7I,CODE_L7Q,
	CODE_L7X,CODE_L8I,CODE_L8Q,CODE_L8X,CODE_L6B,CODE_L6C,CODE_L6X,       0}, /* GAL */
  {	CODE_L2I,CODE_L2Q,CODE_L2X,CODE_L6I,CODE_L6Q,CODE_L6X,CODE_L7I,CODE_L7Q,
	CODE_L7X,       0,       0,       0,       0,       0,       0,       0}, /* BDS */
  {	CODE_L1C,CODE_L1D,CODE_L1P,CODE_L1X,CODE_L2S,CODE_L2L,CODE_L2X,CODE_L5I,
	CODE_L5Q,CODE_L5X,CODE_L6S,CODE_L6L,CODE_L6E,CODE_L1E,       0,       0}, /* QZS */
  {	CODE_L1C,CODE_L5I,CODE_L5Q,CODE_L5X,       0,       0,       0,       0,
		   0,       0,       0,       0,       0,       0,       0,       0}, /* SBS */
  {	CODE_L1D,CODE_L1P,CODE_L1X,CODE_L5A,       0,       0,CODE_L9A,       0,
	       0,       0,       0,       0,       0,       0,       0,       0}, /* IRN */
  {	CODE_L2I,CODE_L2Q,CODE_L2X,CODE_L6I,CODE_L6Q,CODE_L6X,CODE_L7D,CODE_L7P,
	CODE_L7Z,CODE_L1D,CODE_L1P,CODE_L1X,CODE_L5D,CODE_L5P,CODE_L5X,       0}, /* BDS3 */
};

static double decode_sval(unsigned char *buff, int i, int n, double lsb)
{
    int slim=-((1<<(n-1))-1)-1,v;
    v = getbits(buff,i,n);
    return (v==slim) ? INVALID_VALUE:(double)v*lsb;
}

/* system id (rtklib) -> gnss id (cssr) */
static int sys2gnss(int sys, int *prn_min)
{
    int id = CSSR_SYS_NONE;

	if (prn_min) *prn_min = 1;

    switch (sys) {
        case SYS_GPS: id = CSSR_SYS_GPS; break;
        case SYS_GLO: id = CSSR_SYS_GLO; break;
        case SYS_GAL: id = CSSR_SYS_GAL; break;
        case SYS_CMP: id = CSSR_SYS_BDS; break;
        case SYS_SBS:
            id = CSSR_SYS_SBS;
			if (prn_min) *prn_min = 120;
            break;
        case SYS_QZS:
            id = CSSR_SYS_QZS;
			if (prn_min) *prn_min = 193;
            break;
    }
    return id;
}

/* convert GNSS ID of cssr to system id of rtklib */
static int gnss2sys(int id, int *prn_min)
{
    int sys = SYS_NONE;

	if (prn_min) *prn_min = 1;

    switch (id) {
        case CSSR_SYS_GPS: sys = SYS_GPS; break;
        case CSSR_SYS_GLO: sys = SYS_GLO; break;
        case CSSR_SYS_GAL: sys = SYS_GAL; break;
        case CSSR_SYS_BDS: sys = SYS_CMP; break;
        case CSSR_SYS_SBS:
            sys = SYS_SBS;
			if (prn_min) *prn_min = 120;
            break;
        case CSSR_SYS_QZS:
            sys = SYS_QZS;
			if (prn_min) *prn_min = 193;
            break;
    }
    return sys;
}

/* decode cssr header */
static int decode_cssr_head(rtcm_t *rtcm, int *udi, int *sync, int *iod, int i0)
{
	int i=i0+16,dtow,week;
	double tow0,tow,dt;
    gtime_t t;

	tow=time2gpst(rtcm->time,&week);
	if (rtcm->subtype==CSSR_TYPE_MASK) {
		tow0  =getbitu(rtcm->buff,i,20); i+=20;
		rtcm->ssrp.t0 = gpst2time(week,tow0);
		dt=timediff(rtcm->ssrp.t0,rtcm->time);
		if (dt>302400.0)
			rtcm->ssrp.t0=timeadd(rtcm->ssrp.t0,-6048000.0);
		else if (dt<-302400.0)
			rtcm->ssrp.t0=timeadd(rtcm->ssrp.t0, 6048000.0);
	} else {
		dtow  =getbitu(rtcm->buff,i,12); i+=12;
		tow0=floor(time2gpst(rtcm->ssrp.t0,&week)/3600.0)*3600.0;
		t = gpst2time(week,tow0);
		dt=timediff(t,rtcm->ssrp.t0);
		if (dt>1800.0)
		   rtcm->ssrp.t0=timeadd(t,-3600.0);
		else if (dt<-1800.0)
		   rtcm->ssrp.t0=timeadd(t, 3600.0);
    }
	*udi  =getbitu(rtcm->buff,i, 4); i+= 4;
	*sync =getbitu(rtcm->buff,i, 1); i+= 1;
	*iod  =getbitu(rtcm->buff,i, 4); i+= 4;
    return i;
}

/* decode mask message MT4073,1 */
static int decode_cssr_mask(rtcm_t *rtcm, int i0)
{
	int i=i0+16,j,k,udi,sync,iod,ngnss,svid[40],sigs[16],n,nsig,nsat=0,gnss,sys;
	int ii,cmi,ncell,cells[16],tow,sat,idx;
	const int ofst_t[]={1,1,1,1,193,120,1,19};
	ssr_t *ssr=NULL;

    i=decode_cssr_head(rtcm,&udi,&sync,&iod,i0);
	ngnss=getbitu(rtcm->buff,i, 4); i+= 4;

	for (k=0,n=0;k<ngnss;k++) {
		gnss  =getbitu(rtcm->buff,i, 4); i+= 4;
		sys=gnss2sys(gnss,NULL);
		nsat=decode_mask(rtcm->buff,i,40,ofst_t[gnss],svid); i+=40;
		nsig=decode_mask(rtcm->buff,i,16,0,sigs); i+=16;
		cmi=getbitu(rtcm->buff,i,1); i+=1;
		for (j=0;j<nsat;j++) { /* satellite loop */
			sat=satno(sys,svid[j]);
			ssr=&rtcm->ssr[sat-1];
			rtcm->ssrp.sat[n++]=sat;
			if (cmi) {
				ncell=decode_mask(rtcm->buff,i,nsig,0,cells); i+=nsig;
			} else
				ncell=nsig;
			for (ii=0;ii<ncell;ii++) {
                idx=cmi?cells[ii]:ii;
				ssr->ctype[ii]=ssr->ptype[ii]=sig_tbl[gnss][sigs[idx]];
			}
			ssr->nsigc=ssr->nsigp=ncell;
		}
	}
	rtcm->ssrp.iod  = iod;
    rtcm->ssrp.nsat = n;
	rtcm->nbit = i;
    return sync?0:10;
}

/* decode orbit correction MT4073.2 */
static int decode_cssr_oc(rtcm_t *rtcm, int i0)
{
	int i=i0+16,k,j,dtow,udi,sync,iod,sat,sz,sys;
    ssr_t *ssr=NULL;

    i=decode_cssr_head(rtcm,&udi,&sync,&iod,i0);

	for (k=0;k<rtcm->ssrp.nsat;k++) {
		sat=rtcm->ssrp.sat[k];
		sys=satsys(sat,NULL);
		sz=(sys==SYS_GAL)?10:8;
		ssr=&rtcm->ssr[sat-1];
		ssr->iode=getbitu(rtcm->buff,i, sz);i+=sz;
		ssr->deph[0]=decode_sval(rtcm->buff,i,15,0.0016);i+=15;
		ssr->deph[1]=decode_sval(rtcm->buff,i,13,0.0064);i+=13;
		ssr->deph[2]=decode_sval(rtcm->buff,i,13,0.0064);i+=13;
		for (j=0;j<3;j++) ssr->ddeph[j]=0.0;

		ssr->t0 [0]=rtcm->time;
		ssr->udi[0]=udi;
		ssr->iod[0]=iod;
        ssr->update=1;
	}

	rtcm->nbit=i;
	return sync?0:10;
}

/* decode clock correction MT4073.3 */
static int decode_cssr_cc(rtcm_t *rtcm, int i0)
{
	int i=i0+16,k,dtow,udi,sync,sat,iod;
    ssr_t *ssr=NULL;

    i=decode_cssr_head(rtcm,&udi,&sync,&iod,i0);

	for (k=0;k<rtcm->ssrp.nsat;k++) {
		sat=rtcm->ssrp.sat[k];
		ssr=&rtcm->ssr[sat-1];
		ssr->dclk[0]=decode_sval(rtcm->buff,i,15,0.0016);i+=15;
		ssr->dclk[1]=ssr->dclk[2]=0.0;

		ssr->t0 [1]=rtcm->time;
		ssr->udi[1]=udi;
		ssr->iod[1]=iod;
		ssr->update=1;
	}

	rtcm->nbit=i;
	return sync?0:10;
}

/* decode code bias message MT4073.4 */
static int decode_cssr_cb(rtcm_t *rtcm, int i0)
{
	int i=i0+16,k,j,dtow,udi,sync,sat,iod;
	ssr_t *ssr=NULL;

    i=decode_cssr_head(rtcm,&udi,&sync,&iod,i0);

	for (k=0;k<rtcm->ssrp.nsat;k++) {
		sat=rtcm->ssrp.sat[k];
		ssr=&rtcm->ssr[sat-1];
		for (j=0;j<ssr->nsigc;j++) {
			ssr->cbias[ssr->ctype[j]]=decode_sval(rtcm->buff,i,11,0.02);i+=11;
		}
		ssr->t0 [4]=rtcm->time;
		ssr->udi[4]=udi;
		ssr->iod[4]=iod;
		ssr->update=1;
	}
	rtcm->nbit=i;
	return sync?0:10;
}

/* decode phase bias message MT4073.5 */
static int decode_cssr_pb(rtcm_t *rtcm, int i0)
{
	int i=i0+16,k,j,dtow,udi,sync,sat,iod;
	ssr_t *ssr=NULL;

	i=decode_cssr_head(rtcm,&udi,&sync,&iod,i0);

	for (k=0;k<rtcm->ssrp.nsat;k++) {
		sat=rtcm->ssrp.sat[k];
		ssr=&rtcm->ssr[sat-1];
		for (j=0;j<ssr->nsigp;j++) {
			ssr->pbias[ssr->ptype[j]]=decode_sval(rtcm->buff,i,15,1e-3);i+=15;
			ssr->disc[ssr->ptype[j]]=getbitu(rtcm->buff,i,2); i+=2;
		}
		ssr->t0 [5]=rtcm->time;
		ssr->udi[5]=udi;
		ssr->iod[5]=iod;
		ssr->update=1;
	}
	rtcm->nbit=i;
	return sync?0:10;
}

/* code/phase bias correction MT4073.10 */
static int decode_cssr_bias(rtcm_t *rtcm, int i0)
{
	int i=i0+16,k,j,dtow,udi,iod,sync,sat,cbi,pbi,nci,gid,nsat,idx[40];
	ssr_t *ssr=NULL;

	i=decode_cssr_head(rtcm,&udi,&sync,&iod,i0);

	cbi  =getbitu(rtcm->buff,i, 1); i+= 1;
	pbi  =getbitu(rtcm->buff,i, 1); i+= 1;
	nci  =getbitu(rtcm->buff,i, 1); i+= 1;
	if (nci) {
    	gid=getbitu(rtcm->buff,i, 5); i+= 5;
		nsat=decode_mask(rtcm->buff,i,rtcm->ssrp.nsat,0,idx);
		i+=rtcm->ssrp.nsat;
	} else
		nsat=rtcm->ssrp.nsat;

	for (k=0;k<nsat;k++) {
		sat=rtcm->ssrp.sat[idx[k]];
		ssr=&rtcm->ssr[sat-1];
		for (j=0;j<ssr->nsigp;j++) {
			if (cbi)
				ssr->cbias[ssr->ctype[j]]=decode_sval(rtcm->buff,i,11,0.02);i+=11;
			if (pbi) {
				ssr->pbias[ssr->ptype[j]]=decode_sval(rtcm->buff,i,15,1e-3);i+=15;
				ssr->disc[ssr->ptype[j]]=getbitu(rtcm->buff,i,2); i+=2;
			}
		}
		ssr->t0 [4]=ssr->t0 [5]=rtcm->time;
		ssr->udi[4]=ssr->udi[5]=udi;
		ssr->iod[4]=ssr->iod[5]=iod;
		ssr->update=1;
	}
	rtcm->nbit=i;
	return sync?0:10;
}

/* decode ura correction MT4073.7 */
static int decode_cssr_ura(rtcm_t *rtcm, int i0)
{
	int i=i0+16,k,j,dtow,udi,sync,iod,sat,nsat;
	ssr_t *ssr=NULL;

	i=decode_cssr_head(rtcm,&udi,&sync,&iod,i0);

	for (k=0;k<rtcm->ssrp.nsat;k++) {
		sat=rtcm->ssrp.sat[k];
		ssr=&rtcm->ssr[sat-1];
		ssr->ura = getbitu(rtcm->buff,i, 6); i+= 6; /* ssr ura */
		ssr->t0 [3]=rtcm->time;
		ssr->udi[3]=udi;
		ssr->iod[3]=iod;
		ssr->update=1;
	}
	rtcm->nbit=i;
	return sync?0:10;
}

/* decode stec correction MT4073.8 */
static int decode_cssr_stec(rtcm_t *rtcm, int i0)
{
	int i=i0+16,k,j,dtow,udi,sync,iod,sat,nsat,gid,type,idx[40],qi;
	ssr_t *ssr=NULL;
	ssrprm_t *ssrp=&rtcm->ssrp;
	stec_t *p=&rtcm->ssrp.stec;

	i=decode_cssr_head(rtcm,&udi,&sync,&iod,i0);

   	type =getbitu(rtcm->buff,i, 2); i+= 2;
	gid  =getbitu(rtcm->buff,i, 5); i+= 5;
    nsat =decode_mask(rtcm->buff,i,ssrp->nsat,0,idx); i+=ssrp->nsat;

	for (k=0;k<nsat;k++) {
		sat=ssrp->sat[idx[k]];
		ssr=&rtcm->ssr[sat-1];
		qi = getbitu(rtcm->buff,i, 6); i+= 6;
		p->c[sat-1][0]=decode_sval(rtcm->buff,i,14,0.05);i+=14;
		if(type>0) {
			p->c[sat-1][1]=decode_sval(rtcm->buff,i,12,0.02);i+=12;
			p->c[sat-1][2]=decode_sval(rtcm->buff,i,12,0.02);i+=12;
		}
		if(type>1)
			p->c[sat-1][3]=decode_sval(rtcm->buff,i,10,0.02);i+=10;
		if(type>2) {
			p->c[sat-1][4]=decode_sval(rtcm->buff,i, 8,5e-3);i+= 8;
			p->c[sat-1][5]=decode_sval(rtcm->buff,i, 8,5e-3);i+= 8;
		}
		ssr->update=1;
	}
	rtcm->nbit=i;
	return sync?0:10;
}

/* decode grid correction MT4073.9 */
static int decode_cssr_grid(rtcm_t *rtcm, int i0)
{
	int i=i0+16,k,j,dtow,udi,sync,iod,sat,nsat,gid,type,idx[40],qi,sz,ng,szi;
    double hs,w,stec;
	ssr_t *ssr=NULL;
	ssrprm_t *ssrp=&rtcm->ssrp;
    stec_t *p=&rtcm->ssrp.stec;

	i=decode_cssr_head(rtcm,&udi,&sync,&iod,i0);

	type =getbitu(rtcm->buff,i, 2); i+= 2; /* trop delay type */
	szi  =getbitu(rtcm->buff,i, 1); i+= 1;
    sz   =(szi==1)?16:7;
	gid  =getbitu(rtcm->buff,i, 5); i+= 5;
	nsat =decode_mask(rtcm->buff,i,ssrp->nsat,0,idx); i+=ssrp->nsat;
	if (type>0) {
		qi=getbitu(rtcm->buff,i,6); i+= 6;
	}
    ng  =getbitu(rtcm->buff,i,  6); i+= 6;

	for (k=0;k<ng;k++) {
		sat=ssrp->sat[idx[k]];
		ssr=&rtcm->ssr[sat-1];
		if (type==1) {
			hs  =decode_sval(rtcm->buff,i, 9,4e-3)+2.3; i+= 9;
			w   =decode_sval(rtcm->buff,i, 8,4e-3); i+= 8;
		}
		for (j=0;j<nsat;j++) {
			sat=ssrp->sat[idx[j]];
			p->r[k][sat-1]=decode_sval(rtcm->buff,i,sz,0.04); i+= sz;
		}
		ssr->update=1;
	}
	rtcm->nbit=i;
	return sync?0:10;
}

/* decode orbit/clock combination message MT4073.10 */
static int decode_cssr_occ(rtcm_t *rtcm, int i0)
{
	int i=i0+16,k,j,sys,dtow,udi,sync,iod,sat,cbi,nci,oi,ci,gid,nsat,idx[40];
    int sz;
	ssr_t *ssr=NULL;

	i=decode_cssr_head(rtcm,&udi,&sync,&iod,i0);

	oi   =getbitu(rtcm->buff,i, 1); i+= 1;
	ci   =getbitu(rtcm->buff,i, 1); i+= 1;
	nci  =getbitu(rtcm->buff,i, 1); i+= 1;
	if (nci) {
    	gid=getbitu(rtcm->buff,i, 5); i+= 5;
		nsat=decode_mask(rtcm->buff,i,rtcm->ssrp.nsat,0,idx);
		i+=rtcm->ssrp.nsat;
	} else {
		nsat=rtcm->ssrp.nsat;
	}

	for (k=0;k<nsat;k++) {
		sat=rtcm->ssrp.sat[idx[k]];
		sys=satsys(sat,NULL);
		ssr=&rtcm->ssr[sat-1];
		if (oi) {
			sz=(sys==SYS_GAL)?10:8;
			ssr->iode=getbitu(rtcm->buff,i, sz);i+=sz;
			ssr->deph[0]=decode_sval(rtcm->buff,i,15,0.0016);i+=15;
			ssr->deph[1]=decode_sval(rtcm->buff,i,13,0.0064);i+=13;
			ssr->deph[2]=decode_sval(rtcm->buff,i,13,0.0064);i+=13;
			for (j=0;j<3;j++) ssr->ddeph[j]=0.0;

			ssr->t0 [0]=rtcm->time;
			ssr->udi[0]=udi;
			ssr->iod[0]=iod;
		}
		if (ci) {
			ssr->dclk[0]=decode_sval(rtcm->buff,i,15,0.0016);i+=15;
			ssr->dclk[1]=ssr->dclk[2]=0.0;
			ssr->t0 [1]=rtcm->time;
			ssr->udi[1]=udi;
			ssr->iod[1]=iod;
		}
		if (ci|oi) {
			ssr->update=1;
		}
	}
	rtcm->nbit=i;
	return sync?0:10;
}

/* decode atmospheric correction message MT4073.12 */
static int decode_cssr_atmos(rtcm_t *rtcm, int i0)
{
	int i=i0+16,k,j,dtow,udi,sync,iod,sat,nsat,gid,type,idx[40],qi,sz,ng,szi;
    int tci,sci,ofst;
    double ct[4],r[MAXGP];
	ssr_t *ssr=NULL;
	ssrprm_t *ssrp=&rtcm->ssrp;
	stec_t *p=&rtcm->ssrp.stec;
	const float lsb_t[4] = {0.04f,0.12f,0.16f,0.24f};
	const int sz_t[4] = {4,4,5,7};

    i=decode_cssr_head(rtcm,&udi,&sync,&iod,i0);

	tci  =getbitu(rtcm->buff,i, 2); i+= 2;
	sci  =getbitu(rtcm->buff,i, 2); i+= 2;
	gid  =getbitu(rtcm->buff,i, 5); i+= 5;
    ng   =getbitu(rtcm->buff,i, 6); i+= 6;

	sz   =(szi==1)?16:7;

	/* trop */
	qi   =getbitu(rtcm->buff,i, 6); i+= 6;
	if (tci>>1) { /* hydro-static (functional) term of trop */
		type =getbitu(rtcm->buff,i, 2); i+= 2; /* trop delay type */
		ct[0] =decode_sval(rtcm->buff,i, 9,4e-3)+2.3; i+= 9;
		if (type>0) {
			ct[1] =decode_sval(rtcm->buff,i, 7,4e-3); i+= 7;
			ct[2] =decode_sval(rtcm->buff,i, 7,4e-3); i+= 7;
		}
		if (type>1)
			ct[3] =decode_sval(rtcm->buff,i, 7,4e-3); i+= 7;
	}
	if (tci&1) { /* wet (residual) term of trop */
		szi  =getbitu(rtcm->buff,i, 1); i+= 1;
		ofst =getbitu(rtcm->buff,i, 4); i+= 4;
		sz=(szi)?8:6;
		for (k=0;k<ng;k++) {
			r[k] =decode_sval(rtcm->buff,i,sz,4e-3); i+= sz;
		}
	}
	/* iono */
	nsat =decode_mask(rtcm->buff,i,ssrp->nsat,0,idx); i+=ssrp->nsat;
	for (j=0;j<nsat;j++) {
		sat=ssrp->sat[idx[j]];
		ssr=&rtcm->ssr[sat-1];
		qi   =getbitu(rtcm->buff,i, 6); i+= 6;
		if (sci>>1) { /* functional term of stec */
			type =getbitu(rtcm->buff,i, 2); i+= 2;
			p->c[sat-1][0]=decode_sval(rtcm->buff,i,14,0.05);i+=14;
			if(type>0) {
				p->c[sat-1][1]=decode_sval(rtcm->buff,i,12,0.02);i+=12;
				p->c[sat-1][2]=decode_sval(rtcm->buff,i,12,0.02);i+=12;
			}
			if(type>1)
				p->c[sat-1][3]=decode_sval(rtcm->buff,i,10,0.02);i+=10;
			if(type>2) {
				p->c[sat-1][4]=decode_sval(rtcm->buff,i, 8,5e-3);i+= 8;
				p->c[sat-1][5]=decode_sval(rtcm->buff,i, 8,5e-3);i+= 8;
			}
		}
		if (sci&1) { /* grid term of stec */
			szi=getbitu(rtcm->buff,i, 2); i+= 2;
			sz =sz_t[szi];
			for (k=0;k<ng;k++) {
				p->r[k][sat-1]=decode_sval(rtcm->buff,i,sz,lsb_t[szi]); i+= sz;
			}
		}
	}
	rtcm->nbit=i;
	return sync?0:10;
}

/* decode service information message MT4073.10 */
static int decode_cssr_si(rtcm_t *rtcm, int i0)
{
	int i=i0+16,j,sync,cnt,sz;
    uint64_t buff[4];

    sync = getbitu(rtcm->buff,i,1); i+=1; /* multiple message indicator */
	cnt = getbitu(rtcm->buff,i,3); i+=3;  /* information message counter */
	sz = getbitu(rtcm->buff,i,2); i+=2; /* data size */

    for (j=0;j<sz;j++) {
		buff[j] = (uint64_t)getbitu(rtcm->buff,i,8)<<32; i+=8;
        buff[j] |= getbitu(rtcm->buff,i,32); i+=32;
    }
    rtcm->nbit=i;
    return sync?0:10;
}

/* decode type 4073: Melco proprietary messages */
extern int decode_cssr(rtcm_t *rtcm, int head)
{
    int i,ret=0;

	i=(head)?24:0;
    rtcm->subtype = getbitu(rtcm->buff,i+12,4);
    trace(3,"decode_cssr subtype=%d\n",rtcm->subtype);

    switch (rtcm->subtype) {
		case CSSR_TYPE_MASK: return decode_cssr_mask(rtcm, i);
		case CSSR_TYPE_OC:	 return decode_cssr_oc(rtcm, i);
		case CSSR_TYPE_CC:	 return decode_cssr_cc(rtcm, i);
		case CSSR_TYPE_CB:	 return decode_cssr_cb(rtcm, i);
		case CSSR_TYPE_PB:	 return decode_cssr_pb(rtcm, i);
		case CSSR_TYPE_BIAS: return decode_cssr_bias(rtcm, i);
		case CSSR_TYPE_URA:	 return decode_cssr_ura(rtcm, i);
		case CSSR_TYPE_STEC: return decode_cssr_stec(rtcm, i);
		case CSSR_TYPE_GRID: return decode_cssr_grid(rtcm, i);
		case CSSR_TYPE_OCC:  return decode_cssr_occ(rtcm, i);
		case CSSR_TYPE_ATMOS:return decode_cssr_atmos(rtcm, i);
		case CSSR_TYPE_SI: 	 return decode_cssr_si(rtcm, i);
		default: return -1;
    }
}

