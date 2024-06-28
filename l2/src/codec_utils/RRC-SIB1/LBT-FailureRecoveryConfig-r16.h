/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_LBT_FailureRecoveryConfig_r16_H_
#define	_LBT_FailureRecoveryConfig_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum LBT_FailureRecoveryConfig_r16__lbt_FailureInstanceMaxCount_r16 {
	LBT_FailureRecoveryConfig_r16__lbt_FailureInstanceMaxCount_r16_n4	= 0,
	LBT_FailureRecoveryConfig_r16__lbt_FailureInstanceMaxCount_r16_n8	= 1,
	LBT_FailureRecoveryConfig_r16__lbt_FailureInstanceMaxCount_r16_n16	= 2,
	LBT_FailureRecoveryConfig_r16__lbt_FailureInstanceMaxCount_r16_n32	= 3,
	LBT_FailureRecoveryConfig_r16__lbt_FailureInstanceMaxCount_r16_n64	= 4,
	LBT_FailureRecoveryConfig_r16__lbt_FailureInstanceMaxCount_r16_n128	= 5
} e_LBT_FailureRecoveryConfig_r16__lbt_FailureInstanceMaxCount_r16;
typedef enum LBT_FailureRecoveryConfig_r16__lbt_FailureDetectionTimer_r16 {
	LBT_FailureRecoveryConfig_r16__lbt_FailureDetectionTimer_r16_ms10	= 0,
	LBT_FailureRecoveryConfig_r16__lbt_FailureDetectionTimer_r16_ms20	= 1,
	LBT_FailureRecoveryConfig_r16__lbt_FailureDetectionTimer_r16_ms40	= 2,
	LBT_FailureRecoveryConfig_r16__lbt_FailureDetectionTimer_r16_ms80	= 3,
	LBT_FailureRecoveryConfig_r16__lbt_FailureDetectionTimer_r16_ms160	= 4,
	LBT_FailureRecoveryConfig_r16__lbt_FailureDetectionTimer_r16_ms320	= 5
} e_LBT_FailureRecoveryConfig_r16__lbt_FailureDetectionTimer_r16;

/* LBT-FailureRecoveryConfig-r16 */
typedef struct LBT_FailureRecoveryConfig_r16 {
	long	 lbt_FailureInstanceMaxCount_r16;
	long	 lbt_FailureDetectionTimer_r16;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} LBT_FailureRecoveryConfig_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_lbt_FailureInstanceMaxCount_r16_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_lbt_FailureDetectionTimer_r16_9;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_LBT_FailureRecoveryConfig_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_LBT_FailureRecoveryConfig_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_LBT_FailureRecoveryConfig_r16_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _LBT_FailureRecoveryConfig_r16_H_ */
#include <asn_internal.h>
