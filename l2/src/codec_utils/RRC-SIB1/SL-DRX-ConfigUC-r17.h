/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SL_DRX_ConfigUC_r17_H_
#define	_SL_DRX_ConfigUC_r17_H_


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
typedef enum SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17_PR {
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17_PR_NOTHING,	/* No components present */
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17_PR_subMilliSeconds,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17_PR_milliSeconds
} SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17_PR;
typedef enum SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds {
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms1	= 0,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms2	= 1,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms3	= 2,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms4	= 3,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms5	= 4,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms6	= 5,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms8	= 6,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms10	= 7,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms20	= 8,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms30	= 9,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms40	= 10,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms50	= 11,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms60	= 12,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms80	= 13,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms100	= 14,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms200	= 15,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms300	= 16,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms400	= 17,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms500	= 18,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms600	= 19,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms800	= 20,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms1000	= 21,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms1200	= 22,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_ms1600	= 23,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_spare8	= 24,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_spare7	= 25,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_spare6	= 26,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_spare5	= 27,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_spare4	= 28,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_spare3	= 29,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_spare2	= 30,
	SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds_spare1	= 31
} e_SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17__milliSeconds;
typedef enum SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17 {
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms0	= 0,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms1	= 1,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms2	= 2,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms3	= 3,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms4	= 4,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms5	= 5,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms6	= 6,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms8	= 7,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms10	= 8,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms20	= 9,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms30	= 10,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms40	= 11,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms50	= 12,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms60	= 13,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms80	= 14,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms100	= 15,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms200	= 16,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms300	= 17,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms500	= 18,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms750	= 19,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms1280	= 20,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms1920	= 21,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_ms2560	= 22,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_spare9	= 23,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_spare8	= 24,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_spare7	= 25,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_spare6	= 26,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_spare5	= 27,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_spare4	= 28,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_spare3	= 29,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_spare2	= 30,
	SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17_spare1	= 31
} e_SL_DRX_ConfigUC_r17__sl_drx_InactivityTimer_r17;
typedef enum SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer1_r17 {
	SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer1_r17_sl0	= 0,
	SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer1_r17_sl1	= 1,
	SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer1_r17_sl2	= 2,
	SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer1_r17_sl4	= 3,
	SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer1_r17_spare4	= 4,
	SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer1_r17_spare3	= 5,
	SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer1_r17_spare2	= 6,
	SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer1_r17_spare1	= 7
} e_SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer1_r17;
typedef enum SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer2_r17 {
	SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer2_r17_sl0	= 0,
	SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer2_r17_sl1	= 1,
	SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer2_r17_sl2	= 2,
	SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer2_r17_sl4	= 3,
	SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer2_r17_spare4	= 4,
	SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer2_r17_spare3	= 5,
	SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer2_r17_spare2	= 6,
	SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer2_r17_spare1	= 7
} e_SL_DRX_ConfigUC_r17__sl_drx_HARQ_RTT_Timer2_r17;
typedef enum SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17 {
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_sl0	= 0,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_sl1	= 1,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_sl2	= 2,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_sl4	= 3,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_sl6	= 4,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_sl8	= 5,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_sl16	= 6,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_sl24	= 7,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_sl33	= 8,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_sl40	= 9,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_sl64	= 10,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_sl80	= 11,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_sl96	= 12,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_sl112	= 13,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_sl128	= 14,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_sl160	= 15,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_sl320	= 16,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_spare15	= 17,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_spare14	= 18,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_spare13	= 19,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_spare12	= 20,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_spare11	= 21,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_spare10	= 22,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_spare9	= 23,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_spare8	= 24,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_spare7	= 25,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_spare6	= 26,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_spare5	= 27,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_spare4	= 28,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_spare3	= 29,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_spare2	= 30,
	SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17_spare1	= 31
} e_SL_DRX_ConfigUC_r17__sl_drx_RetransmissionTimer_r17;
typedef enum SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR {
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_NOTHING,	/* No components present */
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms10,
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms20,
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms32,
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms40,
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms60,
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms64,
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms70,
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms80,
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms128,
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms160,
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms256,
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms320,
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms512,
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms640,
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms1024,
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms1280,
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms2048,
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms2560,
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms5120,
	SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR_ms10240
} SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR;

/* SL-DRX-ConfigUC-r17 */
typedef struct SL_DRX_ConfigUC_r17 {
	struct SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17 {
		SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17_PR present;
		union SL_DRX_ConfigUC_r17__sl_drx_onDurationTimer_r17_u {
			long	 subMilliSeconds;
			long	 milliSeconds;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} sl_drx_onDurationTimer_r17;
	long	 sl_drx_InactivityTimer_r17;
	long	*sl_drx_HARQ_RTT_Timer1_r17;	/* OPTIONAL */
	long	*sl_drx_HARQ_RTT_Timer2_r17;	/* OPTIONAL */
	long	 sl_drx_RetransmissionTimer_r17;
	struct SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17 {
		SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_PR present;
		union SL_DRX_ConfigUC_r17__sl_drx_CycleStartOffset_r17_u {
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
	} sl_drx_CycleStartOffset_r17;
	long	 sl_drx_SlotOffset;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SL_DRX_ConfigUC_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_milliSeconds_4;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_drx_InactivityTimer_r17_37;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_drx_HARQ_RTT_Timer1_r17_70;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_drx_HARQ_RTT_Timer2_r17_79;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_drx_RetransmissionTimer_r17_88;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SL_DRX_ConfigUC_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_SL_DRX_ConfigUC_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_SL_DRX_ConfigUC_r17_1[7];

#ifdef __cplusplus
}
#endif

#endif	/* _SL_DRX_ConfigUC_r17_H_ */
#include <asn_internal.h>
