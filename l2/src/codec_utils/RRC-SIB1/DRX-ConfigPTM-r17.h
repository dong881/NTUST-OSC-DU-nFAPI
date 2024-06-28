/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_DRX_ConfigPTM_r17_H_
#define	_DRX_ConfigPTM_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <NativeInteger.h>
#include <constr_CHOICE.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17_PR {
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17_PR_NOTHING,	/* No components present */
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17_PR_subMilliSeconds,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17_PR_milliSeconds
} DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17_PR;
typedef enum DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds {
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms1	= 0,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms2	= 1,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms3	= 2,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms4	= 3,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms5	= 4,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms6	= 5,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms8	= 6,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms10	= 7,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms20	= 8,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms30	= 9,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms40	= 10,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms50	= 11,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms60	= 12,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms80	= 13,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms100	= 14,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms200	= 15,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms300	= 16,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms400	= 17,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms500	= 18,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms600	= 19,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms800	= 20,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms1000	= 21,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms1200	= 22,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_ms1600	= 23,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_spare8	= 24,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_spare7	= 25,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_spare6	= 26,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_spare5	= 27,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_spare4	= 28,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_spare3	= 29,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_spare2	= 30,
	DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds_spare1	= 31
} e_DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17__milliSeconds;
typedef enum DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17 {
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms0	= 0,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms1	= 1,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms2	= 2,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms3	= 3,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms4	= 4,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms5	= 5,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms6	= 6,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms8	= 7,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms10	= 8,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms20	= 9,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms30	= 10,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms40	= 11,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms50	= 12,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms60	= 13,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms80	= 14,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms100	= 15,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms200	= 16,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms300	= 17,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms500	= 18,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms750	= 19,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms1280	= 20,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms1920	= 21,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_ms2560	= 22,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_spare9	= 23,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_spare8	= 24,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_spare7	= 25,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_spare6	= 26,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_spare5	= 27,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_spare4	= 28,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_spare3	= 29,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_spare2	= 30,
	DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17_spare1	= 31
} e_DRX_ConfigPTM_r17__drx_InactivityTimerPTM_r17;
typedef enum DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17 {
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_sl0	= 0,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_sl1	= 1,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_sl2	= 2,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_sl4	= 3,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_sl6	= 4,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_sl8	= 5,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_sl16	= 6,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_sl24	= 7,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_sl33	= 8,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_sl40	= 9,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_sl64	= 10,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_sl80	= 11,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_sl96	= 12,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_sl112	= 13,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_sl128	= 14,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_sl160	= 15,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_sl320	= 16,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_spare15	= 17,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_spare14	= 18,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_spare13	= 19,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_spare12	= 20,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_spare11	= 21,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_spare10	= 22,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_spare9	= 23,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_spare8	= 24,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_spare7	= 25,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_spare6	= 26,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_spare5	= 27,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_spare4	= 28,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_spare3	= 29,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_spare2	= 30,
	DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17_spare1	= 31
} e_DRX_ConfigPTM_r17__drx_RetransmissionTimerDL_PTM_r17;
typedef enum DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR {
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_NOTHING,	/* No components present */
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms10,
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms20,
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms32,
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms40,
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms60,
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms64,
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms70,
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms80,
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms128,
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms160,
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms256,
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms320,
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms512,
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms640,
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms1024,
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms1280,
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms2048,
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms2560,
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms5120,
	DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR_ms10240
} DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR;

/* DRX-ConfigPTM-r17 */
typedef struct DRX_ConfigPTM_r17 {
	struct DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17 {
		DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17_PR present;
		union DRX_ConfigPTM_r17__drx_onDurationTimerPTM_r17_u {
			long	 subMilliSeconds;
			long	 milliSeconds;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} drx_onDurationTimerPTM_r17;
	long	 drx_InactivityTimerPTM_r17;
	long	*drx_HARQ_RTT_TimerDL_PTM_r17;	/* OPTIONAL */
	long	*drx_RetransmissionTimerDL_PTM_r17;	/* OPTIONAL */
	struct DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17 {
		DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_PR present;
		union DRX_ConfigPTM_r17__drx_LongCycleStartOffsetPTM_r17_u {
			long	 ms10;
			long	 ms20;
			long	 ms32;
			long	 ms40;
			long	 ms60;
			long	 ms64;
			long	 ms70;
			long	 ms80;
			long	 ms128;
			long	 ms160;
			long	 ms256;
			long	 ms320;
			long	 ms512;
			long	 ms640;
			long	 ms1024;
			long	 ms1280;
			long	 ms2048;
			long	 ms2560;
			long	 ms5120;
			long	 ms10240;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} drx_LongCycleStartOffsetPTM_r17;
	long	 drx_SlotOffsetPTM_r17;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} DRX_ConfigPTM_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_milliSeconds_4;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_drx_InactivityTimerPTM_r17_37;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_drx_RetransmissionTimerDL_PTM_r17_71;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_DRX_ConfigPTM_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_DRX_ConfigPTM_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_DRX_ConfigPTM_r17_1[6];

#ifdef __cplusplus
}
#endif

#endif	/* _DRX_ConfigPTM_r17_H_ */
#include <asn_internal.h>
