/*------------------------------------------------------------------------------
* septentrio.c : Septentrio Binary Format (SBF) decoder
*
*          Copyright (C) 2020 by Tomoji TAKASU
*
* reference :
*     [1] Septentrio, mosaic-X5 reference guide applicable to version 4.8.0 of
*         the firmware, June 4, 2020
*     [2] Septentrio, PolaRx5 reference guide applicable to version 5.7.0 of
*         the firmware, December 2, 2025
*
* version : $Revision:$
*
* history : 2013/07/17  1.0  begin writing
*           2013/10/24  1.1  GPS L1 working
*           2013/11/02  1.2  modified by TTAKASU
*           2015/01/26  1.3  fix some problems by Jens Reimann
*           2016/02/04  1.4  by Jens Reimann
*                           - added more sanity checks
*                           - added galileon raw decoding
*                           - added usage of decoded SBAS messages for testing
*                           - add QZSS and Compass/Beidou navigation messages
*                           - fixed code and Doppler for 2nd and following frequency
*                           - fixed bug in glonass ephemeris
*                           - fixed decoding of galileo ephemeris
*                           - fixed lost lock indicator
*                           - fixed sbas decoding
*                           - cleanups
*           2016/03/03  1.5 - fixed TOW in SBAS messages
*           2016/03/12  1.6 - respect code priorities
*                           - fixed bug in carrier phase calculation of type2 data
*                           - unify frequency determination
*                           - improve lock handling
*                           - various bug fixes
*           2016/05/25  1.7  rtk_crc24q() -> crc24q() by T.T
*           2016/07/29  1.8  crc24q() -> rtk_crc24q() by T.T
*           2017/04/11  1.9  (char *) -> (signed char *) by T.T
*           2017/09/01  1.10 suppress warnings
*
*           2020/11/30  1.11 rewritten from scratch to support mosaic-X5 [1]
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

#define SBF_SYNC1       0x24    /* SBF block header 1 */
#define SBF_SYNC2       0x40    /* SBF block header 2 */
#define SBF_MAXSIG      40      /* SBF max signal number */

#define SBF_MEASEPOCH   4027    /* SBF GNSS measurements */
#define SBF_MEASEXTRA   4000    /* SBF GNSS measurements extra info */
#define SBF_GPSRAWCA    4017    /* SBF GPS C/A subframe */
#define SBF_GPSRAWL2C   4018    /* SBF GPS L2C subframe */
#define SBF_GPSRAWL5    4019    /* SBF GPS L5 subframe */
#define SBF_GEORAWL1    4020    /* SBF SBAS L1 navigation frame */
#define SBF_GEORAWL5    4021    /* SBF SBAS L5 navitation frame */
#define SBF_GALRAWFNAV  4022    /* SBF Galileo F/NAV navigation page */
#define SBF_GALRAWINAV  4023    /* SBF Galileo I/NAV navigation page */
#define SBF_GALRAWCNAV  4024    /* SBF Galileo C/NAV navigation page */
#define SBF_GLORAWCA    4026    /* SBF GLONASS L1CA or L2CA navigation string */
#define SBF_BDSRAW      4047    /* SBF BDS navigation page */
#define SBF_QZSRAWL1CA  4066    /* SBF QZSS L1C/A subframe */
#define SBF_QZSRAWL2C   4067    /* SBF QZSS L2C subframe */
#define SBF_QZSRAWL5    4068    /* SBF QZSS L5 subframe */
#define SBF_QZSRAWL6    4069    /* SBF QZSS L6 subframe */
#define SBF_NAVICRAW    4093    /* SBF NavIC/IRNSS subframe */
#define SBF_BDSRAWB1C   4218    /* SBF BDS B1C navigation frame */
#define SBF_BDSRAWB2A   4219    /* SBF BDS B2a navigation frame */
#define SBF_GPSRAWL1C   4221    /* SBF GPS L1C subframe */
#define SBF_QZSRAWL1C   4227    /* SBF QZSS L1C subframe */
#define SBF_QZSRAWL1S   4228    /* SBF QZSS L1S subframe */
#define SBF_BDSRAWB2B   4242    /* SBF BDS B2b subframe */
#define SBF_QZSRAWL5S   4246    /* SBF QZSS L5S subframe */
#define SBF_NAVICRAWL1  4262    /* SBF NavIC L1 subframe */
#define SBF_QZSRAWL6D   4270    /* SBF QZSS L6D subframe */
#define SBF_QZSRAWL6E   4271    /* SBF QZSS L6E subframe */
#define SBF_PVTGEODETIC 4007    /* SBF GNSS Geodetic position/velocity/time */
#define SBF_LBANDTRACKERSTATUS 4201    /* SBF L-band signal tracking status */

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t  *)(p)))
static uint16_t U2(uint8_t *p) {uint16_t a; memcpy(&a,p,2); return a;}
static uint32_t U4(uint8_t *p) {uint32_t a; memcpy(&a,p,4); return a;}
static int32_t  I4(uint8_t *p) {int32_t  a; memcpy(&a,p,4); return a;}

/* svid to satellite number ([1] 4.1.9) --------------------------------------*/
static int svid2sat(int svid)
{
	if (svid<= 37) return satno(SYS_GPS,svid);         /* G1-G37 */
	if (svid<= 61) return satno(SYS_GLO,svid-37);      /* R1-R24 */
	if (svid<= 62) return 0; /* glonass unknown slot */
	if (svid<= 68) return satno(SYS_GLO,svid-38);      /* R25-R30 */
	if (svid<= 70) return 0;
	if (svid<=106) return satno(SYS_GAL,svid-70);      /* E1-E36 */
	if (svid<=119) return 0; /* LBand (MSS) satellite */
	if (svid<=140) return satno(SYS_SBS,svid);         /* S20-S40 */
	if (svid<=180) return satno(SYS_CMP,svid-140);     /* C1-C40 */
	if (svid<=190) return satno(SYS_QZS,svid-180+192); /* J1-J10 */
	if (svid<=197) return satno(SYS_IRN,svid-190);     /* I1-I7 */
	if (svid<=215) return satno(SYS_SBS,svid-57);      /* S41-S58 */
	if (svid<=222) return satno(SYS_IRN,svid-208);     /* I8-I14 */
	if (svid<=245) return satno(SYS_CMP,svid-182);     /* C41-C63 */
 	if (svid<=249) return 0;
	if (svid<=251) return satno(SYS_GPS,svid-212);     /* G38-G39 */
	return 0; /* error */
}
/* signal number table ([1] 4.1.10) ------------------------------------------*/
static uint8_t sig_tbl[SBF_MAXSIG+1][2]={ /* system, obs-code */
    {SYS_GPS, CODE_L1C}, /*  0: GPS L1C/A */
    {SYS_GPS, CODE_L1W}, /*  1: GPS L1P */
    {SYS_GPS, CODE_L2W}, /*  2: GPS L2P */
    {SYS_GPS, CODE_L2L}, /*  3: GPS L2C */
    {SYS_GPS, CODE_L5Q}, /*  4: GPS L5 */
    {SYS_GPS, CODE_L1L}, /*  5: GPS L1C */
    {SYS_QZS, CODE_L1C}, /*  6: QZS L1C/A */
    {SYS_QZS, CODE_L2L}, /*  7: QZS L2C */
    {SYS_GLO, CODE_L1C}, /*  8: GLO L1C/A */
    {SYS_GLO, CODE_L1P}, /*  9: GLO L1P */
    {SYS_GLO, CODE_L2P}, /* 10: GLO L2P */
    {SYS_GLO, CODE_L2C}, /* 11: GLO L2C/A */
    {SYS_GLO, CODE_L3Q}, /* 12: GLO L3 */
    {SYS_CMP, CODE_L1P}, /* 13: BDS B1C */
    {SYS_CMP, CODE_L5P}, /* 14: BDS B2a */
    {SYS_IRN, CODE_L5A}, /* 15: IRN L5 */
    {      0,        0}, /* 16: reserved */
    {SYS_GAL, CODE_L1C}, /* 17: GAL E1(L1BC) */
    {      0,        0}, /* 18: reserved */
    {SYS_GAL, CODE_L6C}, /* 19: GAL E6(E6BC) */
    {SYS_GAL, CODE_L5Q}, /* 20: GAL E5a */
    {SYS_GAL, CODE_L7Q}, /* 21: GAL E5b */
    {SYS_GAL, CODE_L8Q}, /* 22: GAL E5 AltBoc */
    {      0,        0}, /* 23: LBand */
    {SYS_SBS, CODE_L1C}, /* 24: SBS L1C/A */
    {SYS_SBS, CODE_L5I}, /* 25: SBS L5 */
    {SYS_QZS, CODE_L5Q}, /* 26: QZS L5 */
    {SYS_QZS, CODE_L6L}, /* 27: QZS L6 */
    {SYS_CMP, CODE_L2I}, /* 28: BDS B1I */
    {SYS_CMP, CODE_L7I}, /* 29: BDS B2I */
    {SYS_CMP, CODE_L6I}, /* 30: BDS B3I */
    {      0,        0}, /* 31: reserved */
    {SYS_QZS, CODE_L1L}, /* 32: QZS L1C */
    {SYS_QZS, CODE_L1Z}, /* 33: QZS L1S */
    {SYS_CMP, CODE_L7D}, /* 34: BDS B2b */
    {      0,        0}, /* 35: reserved */
	{      0,        0}, /* 36: reserved */
	{SYS_IRN, CODE_L1P}, /* 37: IRN L1 */
	{SYS_QZS, CODE_L1E}, /* 38: QZS L1CB */
	{SYS_QZS, CODE_L5P}  /* 39: QZS L5S */
};
/* signal number to freq-index and code --------------------------------------*/
static int sig2idx(int sat, int sig, const char *opt, uint8_t *code)
{
    int idx,sys=satsys(sat,NULL),nex=NEXOBS;
    
    if (sig<0||sig>SBF_MAXSIG||sig_tbl[sig][0]!=sys) return -1;
    *code=sig_tbl[sig][1];
    idx=code2idx(sys,*code);
    
    /* resolve code priority in a freq-index */
    if (sys==SYS_GPS) {
        if (strstr(opt,"-GL1W")&&idx==0) return (*code==CODE_L1W)?0:-1;
        if (strstr(opt,"-GL1L")&&idx==0) return (*code==CODE_L1L)?0:-1;
        if (strstr(opt,"-GL2L")&&idx==1) return (*code==CODE_L2L)?1:-1;
        if (*code==CODE_L1W) return (nex<1)?-1:NFREQ;
        if (*code==CODE_L2L) return (nex<2)?-1:NFREQ+1;
        if (*code==CODE_L1L) return (nex<3)?-1:NFREQ+2;
    }
    else if (sys==SYS_GLO) {
        if (strstr(opt,"-RL1P")&&idx==0) return (*code==CODE_L1P)?0:-1;
        if (strstr(opt,"-RL2C")&&idx==1) return (*code==CODE_L2C)?1:-1;
        if (*code==CODE_L1P) return (nex<1)?-1:NFREQ;
        if (*code==CODE_L2C) return (nex<2)?-1:NFREQ+1;
    }
    else if (sys==SYS_QZS) {
        if (strstr(opt,"-JL1L")&&idx==0) return (*code==CODE_L1L)?0:-1;
        if (strstr(opt,"-JL1Z")&&idx==0) return (*code==CODE_L1Z)?0:-1;
        if (*code==CODE_L1L) return (nex<1)?-1:NFREQ;
		if (*code==CODE_L1E) return (nex<2)?-1:NFREQ+1;
		if (*code==CODE_L1Z) return (nex<3)?-1:NFREQ+2;
	}
    else if (sys==SYS_CMP) {
        if (strstr(opt,"-CL1P")&&idx==0) return (*code==CODE_L1P)?0:-1;
        if (*code==CODE_L1P) return (nex<1)?-1:NFREQ;
    }
    return (idx<NFREQ)?idx:-1;
}
/* initialize obs data fields ------------------------------------------------*/
static void init_obsd(gtime_t time, int sat, obsd_t *data)
{
    int i;
    
    data->time=time;
    data->sat=(uint8_t)sat;
    
    for (i=0;i<NFREQ+NEXOBS;i++) {
        data->L[i]=data->P[i]=0.0;
        data->D[i]=0.0f;
        data->SNR[i]=(uint16_t)0;
        data->LLI[i]=(uint8_t)0;
        data->code[i]=CODE_NONE;
    }
}
/* decode SBF GNSS measurements ----------------------------------------------*/
static int decode_measepoch(raw_t *raw)
{
    uint8_t *p=raw->buff+14,code;
    double P1,P2,L1,L2,D1,D2,S1,S2,freq1,freq2;
    int i,j,idx,n,n1,n2,len1,len2,sig,ant,svid,info,sat,sys,prn,lock,fcn,LLI;
    int ant_sel=0; /* antenna selection (0:main) */
    
    if      (strstr(raw->opt,"-AUX1")) ant_sel=1;
    else if (strstr(raw->opt,"-AUX2")) ant_sel=2;
    
    if (raw->len<20) {
        trace(2,"sbf measepoch length error: len=%d\n",raw->len);
        return -1;
    }
    n1  =U1(p);
    len1=U1(p+1);
    len2=U1(p+2);
    
    if (U1(p+3)&0x80) {
        trace(2,"sbf measepoch scrambled\n");
        return -1;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype)," nsat=%d",n1);
    }
    for (i=n=0,p+=6;i<n1&&n<MAXOBS&&p+20<=raw->buff+raw->len;i++) {
        svid=U1(p+2);
        ant =U1(p+1)>>5;
        sig =U1(p+1)&0x1f;
        info=U1(p+18);
        n2  =U1(p+19);
        fcn =0;
        if (sig==31) sig=(info>>3)+32;
        else if (sig>=8&&sig<=11) fcn=(info>>3)-8;
        
        if (ant!=ant_sel) {
            trace(3,"sbf measepoch ant error: svid=%d ant=%d\n",svid,ant);
            p+=len1+len2*n2;
            continue;
        }
        if (!(sat=svid2sat(svid))) {
            trace(3,"sbf measepoch svid error: svid=%d\n",svid);
            p+=len1+len2*n2;
            continue;
		}
		sys=satsys(sat,&prn);
		if ((idx=sig2idx(sat,sig,raw->opt,&code))<0) {
			trace(2,"sbf measepoch sig error: sys=%d prn=%d sig=%d\n",sys,prn,sig);
			p+=len1+len2*n2;
			continue;
		}
		init_obsd(raw->time,sat,raw->obs.data+n);
		P1=D1=0.0;
		freq1=code2freq(sys,code,fcn);

        if ((U1(p+3)&0x1f)!=0||U4(p+4)!=0) {
            P1=(U1(p+3)&0x0f)*4294967.296+U4(p+4)*0.001;
            raw->obs.data[n].P[idx]=P1;
        }
        if (I4(p+8)!=(int)0x80000000) {
            D1=I4(p+8)*0.0001;
            raw->obs.data[n].D[idx]=(float)D1;
        }
        lock=U2(p+16);
        if (P1!=0.0&&freq1>0.0&&lock!=65535&&(I1(p+14)!=-128||U2(p+12)!=0)) {
            L1=I1(p+14)*65.536+U2(p+12)*0.001;
            raw->obs.data[n].L[idx]=P1*freq1/CLIGHT+L1;
            LLI=(lock<raw->lockt[sat-1][idx]?1:0)+((info&(1<<2))?2:0);
            raw->obs.data[n].LLI[idx]=(uint8_t)LLI;
            raw->lockt[sat-1][idx]=lock;
        }
        if (U1(p+15)!=255) {
            S1=U1(p+15)*0.25+((sig==1||sig==2)?0.0:10.0);
            raw->obs.data[n].SNR[idx]=(uint16_t)(S1/SNR_UNIT+0.5);
        }
        raw->obs.data[n].code[idx]=code;
        
        for (j=0,p+=len1;j<n2&&p+12<=raw->buff+raw->len;j++,p+=len2) {
            sig =U1(p)&0x1f;
            ant =U1(p)>>5;
            info=U1(p+5);
            if (sig==31) sig=(info>>3)+32;
            
            if (ant!=ant_sel) {
                trace(3,"sbf measepoch ant error: sat=%d ant=%d\n",sat,ant);
                continue;
            }
			if ((idx=sig2idx(sat,sig,raw->opt,&code))<0) {
                int prn;
				sys = satsys(sat, &prn);
				trace(3,"sbf measepoch sig error: sys=%d prn=%d sig=%d\n",
						sys,prn,sig);
                continue;
            }
            P2=0.0;
            freq2=code2freq(sys,code,fcn);
            
            if (P1!=0.0&&(getbits(p+3,5,3)!=-4||U2(p+6)!=0)) {
                P2=P1+getbits(p+3,5,3)*65.536+U2(p+6)*0.001;
                raw->obs.data[n].P[idx]=P2;
            }
            if (P2!=0.0&&freq2>0.0&&(I1(p+4)!=-128||U2(p+8)!=0)) {
                L2=I1(p+4)*65.536+U2(p+8)*0.001;
                raw->obs.data[n].L[idx]=P2*freq2/CLIGHT+L2;
            }
            if (D1!=0.0&&freq1>0.0&&freq2>0.0&&
                (getbits(p+3,0,5)!=-16||U2(p+10)!=0)) {
                D2=getbits(p+3,0,5)*6.5536+U2(p+10)*0.0001;
                raw->obs.data[n].D[idx]=(float)((D1*freq2/freq1)+D2);
            }
            lock=U1(p+1);
            if (lock!=255) {
                LLI=(lock<raw->lockt[sat-1][idx]?1:0)+((info&(1<<2))?2:0);
                raw->obs.data[n].LLI[idx]=(uint8_t)LLI;
                raw->lockt[sat-1][idx]=lock;
            }
            if (U1(p+2)!=255) {
                S2=U1(p+2)*0.25+((sig==1||sig==2)?0.0:10.0);
                raw->obs.data[n].SNR[idx]=(uint16_t)(S2/SNR_UNIT+0.5);
            }
            raw->obs.data[n].code[idx]=code;
        }
        n++;
    }
    raw->obs.n=n;
    return 1;
}
/* decode SBF GNSS measurements extra info -----------------------------------*/
static int decode_measextra(raw_t *raw)
{
    /* not yet supported */
    return 0;
}
/* decode ephemeris ----------------------------------------------------------*/
static int decode_eph(raw_t *raw, int sat)
{
    int sys=satsys(sat,NULL);
    eph_t eph={0};
    
    if (!decode_gps_lnav(raw->subfrm[sat-1],&eph,NULL,NULL,NULL,sys)) return 0;
    
    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph[sat-1].iode&&
            eph.iodc==raw->nav.eph[sat-1].iodc&&
            timediff(eph.toe,raw->nav.eph[sat-1].toe)==0.0&&
            timediff(eph.toc,raw->nav.eph[sat-1].toc)==0.0) return 0;
    }
    eph.sat=sat;
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;
    raw->ephset=0;
    return 2;
}
/* decode ION/UTC parameters -------------------------------------------------*/
static int decode_ionutc(raw_t *raw, int sat)
{
    double ion[8],utc[8];
    int sys=satsys(sat,NULL);
    
    if (!decode_gps_lnav(raw->subfrm[sat-1],NULL,NULL,ion,utc,sys)) return 0;
    
    adj_utcweek(raw->time,utc,8);
	if (sys==SYS_QZS) {
		set_ion_param(raw,sat,NAV_QZS_LNAV,ion);
		set_utc_param(raw,sat,NAV_QZS_LNAV,utc);
    }
	else {
		set_ion_param(raw,sat,NAV_GPS_LNAV,ion);
		set_utc_param(raw,sat,NAV_GPS_LNAV,utc);
    }
    return 1;
}
/* decode SBF raw C/A subframe -----------------------------------------------*/
static int decode_rawca(raw_t *raw, int sys)
{
    uint8_t *p=raw->buff+14,buff[30];
    int i,svid,sat,prn,id,ret;
    
    if (raw->len<60) {
        trace(2,"sbf rawca length error: sys=%d len=%d\n",sys,raw->len);
        return -1;
    }
    svid=U1(p);
    if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=sys) {
        trace(2,"sbf rawca svid error: sys=%d svid=%d\n",sys,svid);
        return -1;
    }
    if (!U1(p+1)) {
        trace(3,"sbf rawca parity/crc error: sys=%d prn=%d\n",sys,prn);
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype)," prn=%d",prn);
    }
    for (i=0,p+=6;i<10;i++,p+=4) { /* 24 x 10 bits w/o parity */
        setbitu(buff,24*i,24,U4(p)>>6);
    }
    id=getbitu(buff,43,3);

    if (id<1||id>5) {
        trace(2,"sbf rawca subframe id error: sys=%d prn=%d id=%d\n",sys,prn,id);
        return -1;
    }
    memcpy(raw->subfrm[sat-1]+(id-1)*30,buff,30);

    if (id==3) {
        return decode_eph(raw,sat);
    }
    if (id==4||id==5) {
        ret=decode_ionutc(raw,sat);
        memset(raw->subfrm[sat-1]+id*30,0,30);
        return ret;
    }
    return 0;
}
/* decode SBF GLONASS L1CA or L2CA navigation string -------------------------*/
static int decode_glorawca(raw_t *raw)
{
    geph_t geph={0};
    gtime_t *time;
    double utc[8]={0};
    uint8_t *p=raw->buff+14,buff[12];
    int i,svid,sat,prn,m;
    
    if (raw->len<32) {
        trace(2,"sbf glorawca length error: len=%d\n",raw->len);
        return -1;
    }
    svid=U1(p);
    if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=SYS_GLO) {
        trace(3,"sbf glorawca svid error: svid=%d\n",svid);
        return (svid==62)?0:-1; /* svid=62: slot unknown */
    }
    if (!U1(p+1)) {
        trace(3,"sbf glorawca parity/crc error: prn=%d\n",prn);
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype)," prn=%d",prn);
    }
    for (i=0;i<3;i++) {
        setbitu(buff,32*i,32,U4(p+6+4*i)); /* 85 bits */
    }
    m=getbitu(buff,1,4);
    if (m<1||m>15) {
        trace(2,"sbf glorawca string number error: prn=%d m=%d\n",prn,m);
        return -1;
    }
    time=(gtime_t *)(raw->subfrm[sat-1]+150);
    if (fabs(timediff(raw->time,*time))>30.0) {
        memset(raw->subfrm[sat-1],0,40);
        memcpy(time,&raw->time,sizeof(gtime_t));
    }
    memcpy(raw->subfrm[sat-1]+(m-1)*10,buff,10);
    if (m!=4) return 0;
    
    geph.tof=raw->time;
    if (!decode_glostr(raw->subfrm[sat-1],&geph,utc)) return 0;
    
	/*matcpy(raw->nav.utc_glo,utc,8,1);*/
    /*set_utc_param(raw,sat,NAV_GLO_FDMA,utc);*/

    if (geph.sat!=sat) {
        trace(2,"sbf glorawca satellite error: sat=%d %d\n",sat,geph.sat);
        return -1;
    }
    geph.frq=(int)U1(p+4)-8;
    
    if (!strstr(raw->opt,"-EPHALL")) {
        if (geph.iode==raw->nav.geph[prn-1].iode&&
            timediff(geph.toe,raw->nav.geph[prn-1].toe)==0.0) return 0;
    }
    raw->nav.geph[prn-1]=geph;
    raw->ephsat=sat;
    raw->ephset=0;
    return 2;
}
/* decode SBF Galileo F/NAV navigation page ----------------------------------*/
static int decode_galrawfnav(raw_t *raw)
{
    eph_t eph={0};
    double ion[4]={0},utc[8]={0};
    uint8_t *p=raw->buff+14,buff[32];
    int i,svid,src,sat,prn,type;
    
    if (strstr(raw->opt,"-GALINAV")) return 0;
    
    if (raw->len<52) {
        trace(2,"sbf galrawfnav length error: len=%d\n",raw->len);
        return -1;
    }
    svid=U1(p);
    src =U1(p+3)&0x1f;
    
    if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=SYS_GAL) {
        trace(2,"sbf galrawfnav svid error: svid=%d src=%d\n",svid,src);
        return -1;
    }
    if (!U1(p+1)) {
        trace(3,"sbf galrawfnav parity/crc error: prn=%d src=%d\n",prn,src);
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype)," prn=%d src=%d",prn,src);
    }
    if (src!=20&&src!=22) { /* E5a or E5 AltBOC */
        trace(2,"sbf galrawfnav source error: prn=%d src=%d\n",prn,src);
        return -1;
    }
    for (i=0;i<8;i++) {
        setbitu(buff,32*i,32,U4(p+6+4*i)); /* 244 bits page */
    }
    type=getbitu(buff,0,6); /* page type */
    
    if (type==63) return 0; /* dummy page */
    if (type<1||type>6) {
        trace(2,"sbf galrawfnav page type error: prn=%d type=%d\n",prn,type);
        return -1;
    }
    /* save 244 bits page (31 bytes * 6 page) */
    memcpy(raw->subfrm[sat-1]+SF_OFST_GAL_FNAV+(type-1)*31,buff,31);
    
    if (type!=4) return 0;
    if (!decode_gal_fnav(raw->subfrm[sat-1]+SF_OFST_GAL_FNAV,&eph,ion,utc)) return 0;
    
    if (eph.sat!=sat) {
        trace(2,"sbf galrawfnav satellite error: sat=%d %d\n",sat,eph.sat);
        return -1;
    }
    eph.code|=(1<<1); /* data source: E5a */
    
	adj_utcweek(raw->time,utc,8);
	set_ion_param(raw,sat,NAV_GAL_FNAV,ion);
	set_utc_param(raw,sat,NAV_GAL_FNAV,utc);
    
    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph[sat-1+MAXSAT].iode&&
            timediff(eph.toe,raw->nav.eph[sat-1+MAXSAT].toe)==0.0&&
            timediff(eph.toc,raw->nav.eph[sat-1+MAXSAT].toc)==0.0) return 0;
    }
    raw->nav.eph[sat-1+MAXSAT]=eph;
    raw->ephsat=sat;
    raw->ephset=1; /* 1:F/NAV */
    return 2;
}
/* decode SBF Galileo I/NAV navigation page ----------------------------------*/
static int decode_galrawinav(raw_t *raw)
{
    eph_t eph={0};
    double ion[4]={0},utc[8]={0};
    uint8_t *p=raw->buff+14,buff[32],type,part1,part2,page1,page2;
    int i,j,svid,src,sat,prn;
    
    if (strstr(raw->opt,"-GALFNAV")) return 0;
    
    if (raw->len<52) {
        trace(2,"sbf galrawinav length error: len=%d\n",raw->len);
        return -1;
    }
    svid=U1(p);
    src =U1(p+3)&0x1f;
    
    if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=SYS_GAL) {
        trace(2,"sbf galrawinav svid error: svid=%d src=%d\n",svid,src);
        return -1;
    }
    if (!U1(p+1)) {
        trace(3,"sbf galrawinav parity/crc error: prn=%d src=%d\n",prn,src);
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype)," prn=%d src=%d",prn,src);
    }
    if (src!=17&&src!=21&&src!=22) { /* E1, E5b or E5 AltBOC */
        trace(2,"sbf galrawinav source error: prn=%d src=%d\n",prn,src);
        return -1;
    }
    for (i=0,p+=6;i<8;i++,p+=4) {
        setbitu(buff,32*i,32,U4(p)); /* 114(even) + 120(odd) bits */
    }
    part1=getbitu(buff,  0,1);
    page1=getbitu(buff,  1,1);
    part2=getbitu(buff,114,1);
    page2=getbitu(buff,115,1);

    if (part1!=0||part2!=1) {
        trace(3,"sbf galrawinav part error: prn=%d even/odd=%d %d\n",prn,part1,
              part2);
        return -1;
    }
    if (page1==1||page2==1) return 0; /* alert page */
    
    type=getbitu(buff,2,6); /* word type */
    
    if (type==0||type>6) return 0;
    
    /* save 128 (112:even+16:odd) bits word (16 bytes * 7 word) */
    for (i=0,j=2;i<14;i++,j+=8) {
		raw->subfrm[sat-1][(type-1)*16+i]=getbitu(buff,j,8);
    }
    for (i=14,j=116;i<16;i++,j+=8) {
        raw->subfrm[sat-1][(type-1)*16+i]=getbitu(buff,j,8);
    }
    if (type!=5) return 0;
    if (!decode_gal_inav(raw->subfrm[sat-1],&eph,NULL,NULL)) return 0;
    
    if (eph.sat!=sat) {
        trace(2,"sbf galrawinav satellite error: sat=%d %d\n",sat,eph.sat);
        return -1;
    }
    eph.code|=(src==17)?(1<<0):(1<<2); /* data source: E1 or E5b */

	if(decode_gal_inav(raw->subfrm[sat-1],NULL,ion,NULL)) {
		set_ion_param(raw,sat,NAV_GAL_INAV,ion);
    }
	if(decode_gal_inav(raw->subfrm[sat-1],NULL,NULL,utc)) {
		adj_utcweek(raw->time,utc,8);
		set_utc_param(raw,sat,NAV_GAL_INAV,utc);
	}
    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph[sat-1].iode&&
            timediff(eph.toe,raw->nav.eph[sat-1].toe)==0.0&&
            timediff(eph.toc,raw->nav.eph[sat-1].toc)==0.0) return 0;
    }
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;
    raw->ephset=0; /* 0:I/NAV */
    return 2;
}
/* decode SBF SBAS L1 navigation frame ---------------------------------------*/
static int decode_georawl1(raw_t *raw)
{
    uint8_t *p=raw->buff+14,buff[32];
    int i,svid,sat,prn;
    
    if (raw->len<52) {
        trace(2,"sbf georawl1 length error: len=%d\n",raw->len);
        return -1;
    }
    svid=U1(p);
    if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=SYS_SBS) {
        trace(2,"sbf georawl1 svid error: svid=%d\n",svid);
        return -1;
    }
    if (!U1(p+1)) {
        trace(3,"sbf georawl1 parity/crc error: prn=%d err=%d\n",prn,U1(p+2));
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype)," prn=%d",prn);
    }
    raw->sbsmsg.tow=(int)time2gpst(raw->time,&raw->sbsmsg.week);
	raw->sbsmsg.prn=prn;
    raw->sbsmsg.band=0;/* L1 */
    
    for (i=0;i<8;i++) {
        setbitu(buff,32*i,32,U4(p+6+4*i));
    }
    memcpy(raw->sbsmsg.msg,buff,29); /* 226 bits w/o CRC */
    raw->sbsmsg.msg[28]&=0xC0;
    return 3;
}
/* decode SBF SBAS L5 navigation frame ---------------------------------------*/
static int decode_georawl5(raw_t *raw)
{
    uint8_t *p=raw->buff+14,buff[32];
    int i,svid,sat,prn;

    if (raw->len<52) {
		trace(2,"sbf georawl5 length error: len=%d\n",raw->len);
		return -1;
	}
	svid=U1(p);
	if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=SYS_SBS) {
		trace(2,"sbf georawl5 svid error: svid=%d\n",svid);
		return -1;
	}
	if (!U1(p+1)) {
        trace(3,"sbf georawl5 parity/crc error: prn=%d err=%d\n",prn,U1(p+2));
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype)," prn=%d",prn);
    }
    raw->sbsmsg.tow=(int)time2gpst(raw->time,&raw->sbsmsg.week);
	raw->sbsmsg.prn=prn;
	raw->sbsmsg.band=1;/* L5 */

    for (i=0;i<8;i++) {
        setbitu(buff,32*i,32,U4(p+6+4*i));
    }
    memcpy(raw->sbsmsg.msg,buff,29); /* 226 bits w/o CRC */
    raw->sbsmsg.msg[28]&=0xC0;
    return 3;
}
/* decode SBF BDS navigation frame -------------------------------------------*/
static int decode_bdsraw(raw_t *raw)
{
    eph_t eph={0};
    double ion[8],utc[8];
    uint8_t *p=raw->buff+14,buff[40];
    int i,id,svid,sat,prn,pgn;
    
    if (raw->len<52) {
        trace(2,"sbf bdsraw length error: len=%d\n",raw->len);
        return -1;
    }
    svid=U1(p);
    if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=SYS_CMP) {
        trace(2,"sbf bdsraw svid error: svid=%d\n",svid);
        return -1;
    }
    if (!U1(p+1)) {
        trace(3,"sbf bdsraw parity/crc error: prn=%d\n",prn);
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype)," prn=%d",prn);
    }
    for (i=0,p+=6;i<10;i++,p+=4) {
        setbitu(buff,32*i,32,U4(p));
    }
    id=getbitu(buff,15,3); /* subframe ID */
    if (id<1||id>5) {
        trace(2,"sbf bdsraw id error: prn=%d id=%d\n",prn,id);
        return -1;
    }
    if (prn>=6&&prn<=58) { /* IGSO/MEO */
		memcpy(raw->subfrm[sat-1]+(id-1)*38,buff,38);
        
        if (id==3) {
            if (!decode_bds_d1(raw->subfrm[sat-1],&eph,NULL,NULL)) return 0;
        }
        else if (id==5) {
			if (!decode_bds_d1(raw->subfrm[sat-1],NULL,ion,utc)) return 0;
			set_ion_param(raw,sat,NAV_CMP_D1,ion);
			set_utc_param(raw,sat,NAV_CMP_D1,utc);
            return 9;
        }
        else return 0;
    }
    else { /* GEO */
        pgn=getbitu(buff,42,4); /* page number */
        
        if (id==1&&pgn>=1&&pgn<=10) {
            memcpy(raw->subfrm[sat-1]+(pgn-1)*38,buff,38);
            if (pgn!=10) return 0;
            if (!decode_bds_d2(raw->subfrm[sat-1],&eph,NULL)) return 0;
        }
        else if (id==1&&pgn==102) {
            memcpy(raw->subfrm[sat-1]+10*38,buff,38);
            if (!decode_bds_d2(raw->subfrm[sat-1],NULL,utc)) return 0;
			set_utc_param(raw,sat,NAV_CMP_D2,utc);
            return 9;
        }
        else return 0;
    }
    if (!strstr(raw->opt,"-EPHALL")) {
        if (timediff(eph.toe,raw->nav.eph[sat-1].toe)==0.0) return 0;
    }
    eph.sat=sat;
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;
    raw->ephset=0;
    return 2;
}
/* decode SBF BDS CNAV1 navigation frame -------------------------------------*/
static int decode_bdsrawcnav1(raw_t *raw)
{
    eph_t eph={0};
    double ion[8]={0},utc[8]={0},eop[7]={0};
    uint8_t *p=raw->buff+14,buff[228];
    int i,id,svid,sat,prn;

    if (raw->len<248) {
		trace(2,"sbf bdsrawcnav1 length error: len=%d\n",raw->len);
        return -1;
    }
    svid=U1(p);
    if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=SYS_CMP) {
        trace(2,"sbf bdsrawcnav1 svid error: svid=%d\n",svid);
        return -1;
    }
    if (!U1(p+1)||!U1(p+2)) {
        trace(3,"sbf bdsrawcnav1 parity/crc error: prn=%d\n",prn);
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype)," prn=%d",prn);
    }
    for (i=0,p+=6;i<57;i++,p+=4) {
        setbitu(buff,32*i,32,U4(p));
	}

	if (!decode_bds_cnav1(buff,&eph,NULL,NULL,NULL,0)) return 0;

	id=getbitu(buff,1272,6); /* subframe 3 page ID */
	if (id==1) {  /* iono/UTC */
		if (decode_bds_cnav1(raw->subfrm[sat-1],NULL,ion,NULL,NULL,0)) {
			set_ion_param(raw,sat,NAV_CMP_CNV1,ion);
		}
		if (decode_bds_cnav1(raw->subfrm[sat-1],NULL,NULL,utc,NULL,0)) {
			set_utc_param(raw,sat,NAV_CMP_CNV1,utc);
		}
	} else if (id==3) { /* EOP/BGTO */
		if (decode_bds_cnav1(raw->subfrm[sat-1],NULL,NULL,NULL,eop,0)) {
			set_eop_param(raw,sat,NAV_CMP_CNV1,eop);
        }
		/*return 9;*/
	}

    if (!strstr(raw->opt,"-EPHALL")) {
		if (timediff(eph.toe,raw->nav.eph[sat-1+MAXSAT].toe)==0.0) return 0;
    }
    eph.sat=sat;
	raw->nav.eph[sat-1+MAXSAT]=eph;
	raw->ephsat=sat;
	raw->ephset=1;
    return 2;
}
/* decode SBF BDS CNAV2 navigation frame -------------------------------------*/
static int decode_bdsrawcnav2(raw_t *raw)
{
	eph_t eph={0};
	double ion[8]={0},utc[8]={0},eop[7]={0};
	uint8_t *p=raw->buff+14,buff[72];
	int i,id,svid,sat,prn,ofst;

	if (raw->len<92) {
		trace(2,"sbf bdsrawcnav2 length error: len=%d\n",raw->len);
		return -1;
	}
	svid=U1(p);
	if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=SYS_CMP) {
		trace(2,"sbf bdsrawcnav2 svid error: svid=%d\n",svid);
		return -1;
	}
	if (!U1(p+1)) {
        trace(3,"sbf bdsrawcnav2 parity/crc error: prn=%d\n",prn);
        return 0;
    }
    if (raw->outtype) {
		sprintf(raw->msgtype+strlen(raw->msgtype)," prn=%d",prn);
    }
    for (i=0,p+=6;i<18;i++,p+=4) {
        setbitu(buff,32*i,32,U4(p));
	}

	id=getbitu(buff,6,6); /* frame ID */

	switch (id) {
		case 10: ofst=0;break;
		case 11: ofst=1;break;
		case 30: ofst=2;break;
		case 40: ofst=3;break;
		default: return 0;
	}

	memcpy(raw->subfrm[sat-1]+SF_OFST_BDS_CNAV2+ofst*36,buff,36);

	if (id==30) {
		if (!decode_bds_cnav2(raw->subfrm[sat-1]+SF_OFST_BDS_CNAV2,&eph,
			NULL,NULL,NULL,12)) return 0;
	} else {
        return 0;
    }

    if (!strstr(raw->opt,"-EPHALL")) {
		if (timediff(eph.toe,raw->nav.eph[sat-1+MAXSAT*2].toe)==0.0) return 0;
	}

	if (decode_bds_cnav2(raw->subfrm[sat-1]+SF_OFST_BDS_CNAV2,NULL,
		ion,NULL,NULL,0)) {
		set_ion_param(raw,sat,NAV_CMP_CNV2,ion);
	}
	if (decode_bds_cnav2(raw->subfrm[sat-1]+SF_OFST_BDS_CNAV2,NULL,
		NULL,utc,NULL,0)) {
		set_utc_param(raw,sat,NAV_CMP_CNV2,utc);
	}
	if (decode_bds_cnav2(raw->subfrm[sat-1]+SF_OFST_BDS_CNAV2,NULL,
		NULL,NULL,eop,0)) {
		set_eop_param(raw,sat,NAV_CMP_CNV2,eop);
	}

    eph.sat=sat;
	raw->nav.eph[sat-1+MAXSAT*2]=eph;
    raw->ephsat=sat;
	raw->ephset=2;
    return 2;
}
/* decode SBF BDS CNAV3 navigation frame -------------------------------------*/
static int decode_bdsrawcnav3(raw_t *raw)
{
	eph_t eph={0};
	double ion[8]={0},utc[8]={0},eop[7]={0};
	uint8_t *p=raw->buff+14,buff[124];
	int i,id,svid,sat,prn,ofst;

	if (raw->len<144) {
		trace(2,"sbf bdsrawcnav3 length error: len=%d\n",raw->len);
		return -1;
	}
	svid=U1(p);
	if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=SYS_CMP) {
		trace(2,"sbf bdsrawcnav3 svid error: svid=%d\n",svid);
		return -1;
	}
	if (!U1(p+1)) {
		trace(3,"sbf bdsrawcnav3 parity/crc error: prn=%d\n",prn);
        return 0;
    }
    if (raw->outtype) {
		sprintf(raw->msgtype+strlen(raw->msgtype)," prn=%d",prn);
	}

	if (prn>=58) { /* skip GEO */
		return 0;
	}

    for (i=0,p+=6;i<31;i++,p+=4) {
		setbitu(buff,32*i,32,U4(p));
	}

	if(getbitu(buff,0,6)!=prn) {
		trace(3,"sbf bdsrawcnav3 prn unmatch: prn=%d\n",prn);
        return 0;
    }

	id=getbitu(buff,12,6); /* frame ID */

	switch (id) {
		case 10: ofst=0;break;
		case 30: ofst=1;break;
		default: return 0;
	}

	memcpy(raw->subfrm[sat-1]+SF_OFST_BDS_CNAV3+ofst*64,buff,64);

	if (id==30) {
		if (!decode_bds_cnav3(raw->subfrm[sat-1]+SF_OFST_BDS_CNAV3,&eph,
			NULL,NULL,NULL,0)) return 0;
	} else {
        return 0;
    }

    if (!strstr(raw->opt,"-EPHALL")) {
		if (timediff(eph.toe,raw->nav.eph[sat-1+MAXSAT*3].toe)==0.0) return 0;
	}

	if(decode_bds_cnav3(raw->subfrm[sat-1]+SF_OFST_BDS_CNAV3,NULL,ion,NULL,NULL,0)) {
		set_ion_param(raw,sat,NAV_CMP_CNV3,ion);
	}
	if(decode_bds_cnav3(raw->subfrm[sat-1]+SF_OFST_BDS_CNAV3,NULL,NULL,utc,NULL,0)) {
		set_utc_param(raw,sat,NAV_CMP_CNV3,utc);
	}
	if(decode_bds_cnav3(raw->subfrm[sat-1]+SF_OFST_BDS_CNAV3,NULL,NULL,NULL,eop,0)) {
		set_eop_param(raw,sat,NAV_CMP_CNV3,eop);
	}

    eph.sat=sat;
	raw->nav.eph[sat-1+MAXSAT*3]=eph;
    raw->ephsat=sat;
	raw->ephset=3;
    return 2;
}
/* decode SBF GPS/QZS CNAV(L2C/L5) subframe ----------------------------------*/
static int decode_gpsrawcnav(raw_t *raw, int sys)
{
    uint8_t *p=(raw->buff)+14;
	int i,prn,sat,id,ofst=-1;
	uint8_t buff[40];
	eph_t eph={0};
	double ion[4]={0},utc[8]={0},eop[7]={0};

    if (raw->len!=60)
    {
		trace(3,"SBF decode_gpsrawcnav: Block length mismatch (60) %d\n", raw->len);
		return -1;
    }

	sat=svid2sat(U1(p));
	if (sys!=satsys(sat,&prn)) {
		trace(2,"SBF decode_gpsrawcnav: prn out of range: %3d\n",prn); return 0;
	}
	if (U1(p+1)!=1) {
		trace(2,"SBF decode_gpsrawcnav: crc_pass failed\n"); return 0;
	}
	/* copy data */
	for (i=0,p+=6;i<10;i++,p+=4) {
		setbitu(buff,32*i,32,U4(p));
	}

	id=getbitu(buff, 14, 6);
	switch (id) {
		case 10: ofst=0;break;
		case 11: ofst=1;break;
		case 30:
		case 31:
		case 32:
		case 33:
		case 35:
		case 37:
		case 61: ofst=2;break;
		default: return 0;
	}
	memcpy(raw->subfrm[sat-1]+SF_OFST_GPS_CNAV+ofst*35,buff,35);
	if(!decode_gps_cnav(raw->subfrm[sat-1]+SF_OFST_GPS_CNAV,&eph,NULL,NULL,
		NULL,NULL,sys))
        return 0;

	if (!strstr(raw->opt,"-EPHALL")) {
		if (timediff(eph.toe,raw->nav.eph[sat-1+MAXSAT].toe)==0.0) return 0;
	}

	id=(sys==SYS_GPS)?NAV_GPS_CNV2:NAV_QZS_CNV2;

	if(decode_gps_cnav(raw->subfrm[sat-1]+SF_OFST_GPS_CNAV,NULL,
		NULL,ion,NULL,NULL,sys)) {
		set_ion_param(raw,sat,id,ion);
	}
	if (decode_gps_cnav(raw->subfrm[sat-1]+SF_OFST_GPS_CNAV,NULL,NULL,NULL,
		utc,NULL,sys)) {
		set_utc_param(raw,sat,id,utc);
	}
	if (decode_gps_cnav(raw->subfrm[sat-1]+SF_OFST_GPS_CNAV,NULL,NULL,NULL,
		NULL,eop,sys)) {
		set_eop_param(raw,sat,id,eop);
	}

    eph.sat=sat;
	raw->nav.eph[sat-1+MAXSAT]=eph;
	raw->ephsat=sat;
	raw->ephset=1;

    return 2;
}
/* decode SBF GPS/QZS CNAV2(L1C) subframe ------------------------------------*/
static int decode_gpsrawcnav2(raw_t *raw, int sys)
{
	uint8_t *p=(raw->buff)+8;
	int i,prn,sat,id;
	uint8_t buff[228];
	uint16_t week;
	double tow;
	double ion[4]={0},utc[8]={0},eop[7]={0};
    eph_t eph={0};

	if (raw->len!=248)
	{
		trace(3,"SBF decode_gpsrawcnav2: Block length mismatch (248) %d\n", raw->len);
		return -1;
	}

	tow = U4(p)*0.001;p+=4;
	week = U2(p);p+=2;
	sat=svid2sat(U1(p));
	if (sys!=satsys(sat,&prn)) {
		trace(2,"SBF decode_gpsrawcnav2: prn out of range: %3d\n",prn); return 0;
	}

	if (U1(p+1)!=1||U1(p+2)!=1) {
		trace(2,"SBF decode_gpsrawcnav2: crc_pass failed\n"); return 0;
	}

    /* copy data */
	for (i=0,p+=6;i<57;i++,p+=4) {
		setbitu(buff,32*i,32,U4(p));
	}

	id=(sys==SYS_GPS)?NAV_GPS_CNV2:NAV_QZS_CNV2;

	if (decode_gps_cnav2(buff,NULL,NULL,ion,NULL,NULL,sys,1)) {
		set_ion_param(raw,sat,id,ion);
	}
	if (decode_gps_cnav2(buff,NULL,NULL,NULL,utc,NULL,sys,1)) {
		set_utc_param(raw,sat,id,utc);
	}
	if (decode_gps_cnav2(buff,NULL,NULL,NULL,NULL,eop,sys,1)) {
		set_eop_param(raw,sat,id,eop);
	}

	if (!decode_gps_cnav2(buff,&eph,NULL,NULL,NULL,NULL,sys,0)) return 0;

    if (!strstr(raw->opt,"-EPHALL")) {
		if (timediff(eph.toe,raw->nav.eph[sat-1+MAXSAT*2].toe)==0.0) return 0;
	}

	eph.sat=sat;
	raw->nav.eph[sat-1+MAXSAT*2]=eph;
    raw->ephsat=sat;
    raw->ephset=2;

    return 2;
}
/* decode SBF raw nav message for QZSS L6 */
static int decode_qzsrawl6(raw_t *raw){

    uint8_t *p=(raw->buff)+14;
    uint8_t buff[63*4];
    int i,j,id,prn;
    uint8_t parity,rscnt,ch;
    uint32_t tmp,preamb;

    if (raw->len!=272)
    {
        trace(1,"SBF decode_qzsrawl6: Block length mismatch (272) %d\n", raw->len);
        return -1;
    }

    prn = (U1(p)-180)+192;
    if (prn<MINPRNQZS && prn>MAXPRNQZS) {
        trace(2,"SBF decode_qzsrawl6: sat out of range: %3d\n",prn); return 0;
    }
#if 0
    id=prn-MINPRNQZS;
#else
    id=0;
#endif
    parity = U1(p+1);
    if (parity != 1) {
        trace(2,"SBF decode_qzssrawl6: parity failed\n"); return 0;
    }
    rscnt = U1(p+2);
    ch = U1(p+5);

    /* copy data */
    for (i=0;i<63;i++)
    {
        tmp = U4(p+6+i*4);
        buff[4*i]=(tmp>>24) & 0xff;
        buff[4*i+1]=(tmp>>16) & 0xff;
        buff[4*i+2]=(tmp>>8) & 0xff;
        buff[4*i+3]=(tmp>>0) & 0xff;
    }

    i=0;
    preamb   =getbitu(buff,i,32); i+=32;
    if (preamb!=0x1ACFFC1Du) {
        trace(1,"SBF decode_qzsrawl6: L6 preamble error: preamb=%08X\n",preamb);
        return 0;
    }
    raw->l6msg[id].prn  =getbitu(buff,i, 8); i+= 8;
    raw->l6msg[id].type =getbitu(buff,i, 8); i+= 8;
    raw->l6msg[id].alert=getbitu(buff,i, 1); i+= 1;
#if 0
    for (j=0;j<212;j++) {
        raw->l6msg[id].msg[j]=(uint8_t)getbitu(buff,i,8); i+=8;
    }
    raw->l6msg[id].msg[211]&=0xFE;
#else
    for (j=0,i=0;j<250;j++) {
        raw->l6msg[id].msg[j]=(uint8_t)getbitu(buff,i,8); i+=8;
    }
#endif
    if (raw->outtype) {
        sprintf(raw->msgtype,"QZSRAWL6 (%4d): prn=%3d ch=%02x type=%d alert=%d",
                raw->len,prn,ch,raw->l6msg[id].type,raw->l6msg[id].alert);
    }

    return 5;
}
/* decode SBF NavIC/IRNSS subframe -------------------------------------------*/
static int decode_navicraw(raw_t *raw)
{
    eph_t eph={0};
    double ion[8]={0},utc[9]={0},eop[7]={0};
    uint8_t *p=raw->buff+14,buff[40];
    int i,id,svid,sat,prn,ret=0;
    
    if (raw->len<52) {
        trace(2,"sbf navicraw length error: len=%d\n",raw->len);
        return -1;
    }
    svid=U1(p);
    if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=SYS_IRN) {
        trace(2,"sbf navicraw svid error: svid=%d\n",svid);
        return -1;
    }
    if (!U1(p+1)) {
        trace(3,"sbf navicraw parity/crc error: prn=%d err=%d\n",prn,U1(p+2));
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype)," prn=%d",prn);
    }
    for (i=0,p+=6;i<10;i++,p+=4) {
        setbitu(buff,32*i,32,U4(p));
    }
    id=getbitu(buff,27,2); /* subframe ID (0-3) */
    
    memcpy(raw->subfrm[sat-1]+id*37,buff,37);
    
    if (id==1) { /* subframe 2 */
        if (!decode_irn_nav(raw->subfrm[sat-1],&eph,NULL,NULL,NULL)) return 0;
        
        if (!strstr(raw->opt,"-EPHALL")) {
            if (eph.iode==raw->nav.eph[sat-1].iode&&
                timediff(eph.toe,raw->nav.eph[sat-1].toe)==0.0) {
                return 0;
            }
        }
        eph.sat=sat;
        raw->nav.eph[sat-1]=eph;
        raw->ephsat=sat;
        raw->ephset=0;
        return 2;
    }
    else if (id==2||id==3) { /* subframe 3 or 4 */
        if (decode_irn_nav(raw->subfrm[sat-1],NULL,ion,NULL,NULL)) {
			set_ion_param(raw,sat,NAV_IRN_LNAV,ion);
			ret=9;
		}
		if (decode_irn_nav(raw->subfrm[sat-1],NULL,NULL,NULL,eop)) {
			set_eop_param(raw,sat,NAV_IRN_LNAV,eop);
			ret=9;
		}
		if (decode_irn_nav(raw->subfrm[sat-1],NULL,NULL,utc,NULL)) {
			adj_utcweek(raw->time,utc,10);
            set_ion_param(raw,sat,NAV_IRN_LNAV,utc);
            ret=9;
        }
        memset(raw->subfrm[sat-1]+id*37,0,37);
        return ret;
    }
    return 0;
}
/* decode SBF NavIC L1 navigation frame -------------------------------------*/
static int decode_navicrawl1(raw_t *raw)
{
    eph_t eph={0};
    double ion[8]={0},utc[8]={0},eop[7]={0};
    uint8_t *p=raw->buff+14,buff[228];
    int i,id,svid,sat,prn;

    if (raw->len<248) {
		trace(2,"sbf navicrawl1 length error: len=%d\n",raw->len);
        return -1;
    }
    svid=U1(p);
	if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=SYS_IRN) {
		trace(2,"sbf navicrawl1 svid error: svid=%d\n",svid);
        return -1;
    }
    if (!U1(p+1)||!U1(p+2)) {
        trace(3,"sbf navicrawl1 parity/crc error: prn=%d\n",prn);
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype)," prn=%d",prn);
    }
    for (i=0,p+=6;i<57;i++,p+=4) {
        setbitu(buff,32*i,32,U4(p));
	}

	if (!decode_irn_l1(buff,&eph,NULL,NULL,NULL,0)) return 0;

	id=getbitu(buff,1272,6); /* subframe 3 page ID */
	if (id==17) {  /* UTC */
		if (!decode_irn_l1(buff,NULL,NULL,utc,NULL,0)) return 0;
		set_utc_param(raw,sat,NAV_IRN_L1NV,utc);
        /*return 9;*/
	} else if (id==10) {  /* EOP/iono */
		if (!decode_irn_l1(buff,NULL,ion,NULL,eop,0)) return 0;
		set_ion_param(raw,sat,NAV_IRN_L1NV,ion);
		set_eop_param(raw,sat,NAV_IRN_L1NV,eop);
        /*return 9;*/
	}

    if (!strstr(raw->opt,"-EPHALL")) {
		if (timediff(eph.toe,raw->nav.eph[sat-1+MAXSAT].toe)==0.0) return 0;
    }
    eph.sat=sat;
	raw->nav.eph[sat-1+MAXSAT]=eph;
	raw->ephsat=sat;
	raw->ephset=1;
    return 2;
}
/* decode SBF block ----------------------------------------------------------*/
static int decode_sbf(raw_t *raw)
{
    uint8_t *p=raw->buff;
    uint32_t week,tow;
    char tstr[32];
    int type=U2(p+4)&0x1fff;
    
    if (rtk_crc16(p+4,raw->len-4)!=U2(p+2)) {
        trace(2,"sbf crc error: type=%d len=%d\n",type,raw->len);
        return -1;
    }
    if (raw->len<14) {
        trace(2,"sbf length error: type=%d len=%d\n",type,raw->len);
        return -1;
    }
    tow =U4(p+8);
    week=U2(p+12);
    if (tow==4294967295u||week==65535u) {
        trace(2,"sbf tow/week error: type=%d len=%d\n",type,raw->len);
        return -1;
    }
    raw->time=gpst2time(week,tow*0.001);
    
    if (raw->outtype) {
        time2str(raw->time,tstr,2);
        sprintf(raw->msgtype,"SBF %4d (%4d): %s",type,raw->len,tstr);
    }
    switch (type) {
        case SBF_MEASEPOCH : return decode_measepoch  (raw);
		case SBF_MEASEXTRA : return decode_measextra  (raw);
		case SBF_GPSRAWCA  : return decode_rawca(raw,SYS_GPS);
		case SBF_GLORAWCA  : return decode_glorawca   (raw);
		case SBF_GALRAWFNAV: return decode_galrawfnav (raw);
		case SBF_GALRAWINAV: return decode_galrawinav (raw);
		case SBF_GEORAWL1  : return decode_georawl1   (raw);
		case SBF_BDSRAW    : return decode_bdsraw     (raw);
		case SBF_QZSRAWL1CA: return decode_rawca(raw,SYS_QZS);
		case SBF_NAVICRAW  : return decode_navicraw   (raw);
		case SBF_NAVICRAWL1: return decode_navicrawl1 (raw);
		case SBF_GPSRAWL2C : return decode_gpsrawcnav (raw,SYS_GPS);
		case SBF_GPSRAWL5  : return decode_gpsrawcnav (raw,SYS_GPS);
		case SBF_GPSRAWL1C : return decode_gpsrawcnav2(raw,SYS_GPS);
		case SBF_GEORAWL5  : return decode_georawl5   (raw);
		case SBF_QZSRAWL2C : return decode_gpsrawcnav (raw,SYS_QZS);
		case SBF_QZSRAWL5  : return decode_gpsrawcnav (raw,SYS_QZS);
		case SBF_QZSRAWL1C : return decode_gpsrawcnav2(raw,SYS_QZS);
		case SBF_QZSRAWL1S : return 0;
		case SBF_QZSRAWL5S : return 0;
		case SBF_QZSRAWL6  : return decode_qzsrawl6  (raw);
		case SBF_QZSRAWL6D : return 0;
		case SBF_QZSRAWL6E : return 0;
		case SBF_BDSRAWB1C : return decode_bdsrawcnav1(raw);
		case SBF_BDSRAWB2A : return decode_bdsrawcnav2(raw);
		case SBF_BDSRAWB2B : return decode_bdsrawcnav3(raw);
		case SBF_GALRAWCNAV: return 0;
		case SBF_PVTGEODETIC : return 0;
		case SBF_LBANDTRACKERSTATUS : return 0;
	}
    trace(3,"sbf unsupported message: type=%d\n",type);
    return 0;
}
/* synchronize SBF block header ----------------------------------------------*/
static int sync_sbf(uint8_t *buff, uint8_t data)
{
    buff[0]=buff[1]; buff[1]=data;
    return buff[0]==SBF_SYNC1&&buff[1]==SBF_SYNC2;
}
/* input SBF raw data from stream ----------------------------------------------
* fetch next SBF raw data and input a mesasge from stream
* args   : raw_t *raw       IO  receiver raw data control struct
*          uint_t data      I   stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*
* notes  : supported SBF block (block ID):
*
*           MEASEPOCH(4027), GPSRAWCA(4017), GLORAWCA(4026), GALRAWFNAV(4022),
*           GALRAWINAV(4023), GEORAWL1(4020), BDSRAW(4047), QZSRAWL1CA(4066),
*           NAVICRAW(4093)
*
*          to specify input options for sbf, set raw->opt to the following
*          option strings separated by spaces.
*
*          -EPHALL : input all ephemerides
*          -AUX1   : select antenna Aux1  (default: main)
*          -AUX2   : select antenna Aux2  (default: main)
*          -GL1W   : select 1W for GPS L1 (default: 1C)
*          -GL1L   : select 1L for GPS L1 (default: 1C)
*          -GL2L   : select 2L for GPS L2 (default: 2W)
*          -RL1P   : select 1P for GLO G1 (default: 1C)
*          -RL2C   : select 2C for GLO G2 (default: 2P)
*          -JL1L   : select 1L for QZS L1 (default: 1C)
*          -JL1Z   : select 1Z for QZS L1 (default: 1C)
*          -CL1P   : select 1P for BDS B1 (default: 2I)
*          -GALINAV: select I/NAV for Galileo ephemeris (default: all)
*          -GALFNAV: select F/NAV for Galileo ephemeris (default: all)
*-----------------------------------------------------------------------------*/
extern int input_sbf(raw_t *raw, uint8_t data)
{
    trace(5,"input_sbf: data=%02x\n",data);
    
    if (raw->nbyte==0) {
        if (sync_sbf(raw->buff,data)) raw->nbyte=2;
        return 0;
    }
    raw->buff[raw->nbyte++]=data;
    if (raw->nbyte<8) return 0;
    
    if ((raw->len=U2(raw->buff+6))>MAXRAWLEN) {
        trace(2,"sbf length error: len=%d\n",raw->len);
        raw->nbyte=0;
        return -1;
    }
    if (raw->nbyte<raw->len) return 0;
    raw->nbyte=0;
    
    /* decode SBF block */
    return decode_sbf(raw);
}
/* input SBF raw data from file ------------------------------------------------
* fetch next SBF raw data and input a message from file
* args   : raw_t *raw       IO  receiver raw data control struct
*          FILE  *fp        I   file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
extern int input_sbff(raw_t *raw, FILE *fp)
{
    int i,data;
    
    trace(4,"input_sbff:\n");
    
    if (raw->nbyte==0) {
        for (i=0;;i++) {
            if ((data=fgetc(fp))==EOF) return -2;
            if (sync_sbf(raw->buff,(uint8_t)data)) break;
            if (i>=4096) return 0;
        }
    }
    if (fread(raw->buff+2,1,6,fp)<6) return -2;
    raw->nbyte=8;
    
    if ((raw->len=U2(raw->buff+6))>MAXRAWLEN) {
        trace(2,"sbf length error: len=%d\n",raw->len);
        raw->nbyte=0;
        return -1;
    }
    if (fread(raw->buff+8,raw->len-8,1,fp)<1) return -2;
    raw->nbyte=0;
    
    /* decode SBF block */
    return decode_sbf(raw);
}

