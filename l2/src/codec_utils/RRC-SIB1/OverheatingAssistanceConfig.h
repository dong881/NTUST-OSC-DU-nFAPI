/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_OverheatingAssistanceConfig_H_
#define	_OverheatingAssistanceConfig_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum OverheatingAssistanceConfig__overheatingIndicationProhibitTimer {
	OverheatingAssistanceConfig__overheatingIndicationProhibitTimer_s0	= 0,
	OverheatingAssistanceConfig__overheatingIndicationProhibitTimer_s0dot5	= 1,
	OverheatingAssistanceConfig__overheatingIndicationProhibitTimer_s1	= 2,
	OverheatingAssistanceConfig__overheatingIndicationProhibitTimer_s2	= 3,
	OverheatingAssistanceConfig__overheatingIndicationProhibitTimer_s5	= 4,
	OverheatingAssistanceConfig__overheatingIndicationProhibitTimer_s10	= 5,
	OverheatingAssistanceConfig__overheatingIndicationProhibitTimer_s20	= 6,
	OverheatingAssistanceConfig__overheatingIndicationProhibitTimer_s30	= 7,
	OverheatingAssistanceConfig__overheatingIndicationProhibitTimer_s60	= 8,
	OverheatingAssistanceConfig__overheatingIndicationProhibitTimer_s90	= 9,
	OverheatingAssistanceConfig__overheatingIndicationProhibitTimer_s120	= 10,
	OverheatingAssistanceConfig__overheatingIndicationProhibitTimer_s300	= 11,
	OverheatingAssistanceConfig__overheatingIndicationProhibitTimer_s600	= 12,
	OverheatingAssistanceConfig__overheatingIndicationProhibitTimer_spare3	= 13,
	OverheatingAssistanceConfig__overheatingIndicationProhibitTimer_spare2	= 14,
	OverheatingAssistanceConfig__overheatingIndicationProhibitTimer_spare1	= 15
} e_OverheatingAssistanceConfig__overheatingIndicationProhibitTimer;

/* OverheatingAssistanceConfig */
typedef struct OverheatingAssistanceConfig {
	long	 overheatingIndicationProhibitTimer;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} OverheatingAssistanceConfig_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_overheatingIndicationProhibitTimer_2;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_OverheatingAssistanceConfig;
extern asn_SEQUENCE_specifics_t asn_SPC_OverheatingAssistanceConfig_specs_1;
extern asn_TYPE_member_t asn_MBR_OverheatingAssistanceConfig_1[1];

#ifdef __cplusplus
}
#endif

#endif	/* _OverheatingAssistanceConfig_H_ */
#include <asn_internal.h>
