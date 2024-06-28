/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_UAC_BarringInfoSet_v1700_H_
#define	_UAC_BarringInfoSet_v1700_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum UAC_BarringInfoSet_v1700__uac_BarringFactorForAI3_r17 {
	UAC_BarringInfoSet_v1700__uac_BarringFactorForAI3_r17_p00	= 0,
	UAC_BarringInfoSet_v1700__uac_BarringFactorForAI3_r17_p05	= 1,
	UAC_BarringInfoSet_v1700__uac_BarringFactorForAI3_r17_p10	= 2,
	UAC_BarringInfoSet_v1700__uac_BarringFactorForAI3_r17_p15	= 3,
	UAC_BarringInfoSet_v1700__uac_BarringFactorForAI3_r17_p20	= 4,
	UAC_BarringInfoSet_v1700__uac_BarringFactorForAI3_r17_p25	= 5,
	UAC_BarringInfoSet_v1700__uac_BarringFactorForAI3_r17_p30	= 6,
	UAC_BarringInfoSet_v1700__uac_BarringFactorForAI3_r17_p40	= 7,
	UAC_BarringInfoSet_v1700__uac_BarringFactorForAI3_r17_p50	= 8,
	UAC_BarringInfoSet_v1700__uac_BarringFactorForAI3_r17_p60	= 9,
	UAC_BarringInfoSet_v1700__uac_BarringFactorForAI3_r17_p70	= 10,
	UAC_BarringInfoSet_v1700__uac_BarringFactorForAI3_r17_p75	= 11,
	UAC_BarringInfoSet_v1700__uac_BarringFactorForAI3_r17_p80	= 12,
	UAC_BarringInfoSet_v1700__uac_BarringFactorForAI3_r17_p85	= 13,
	UAC_BarringInfoSet_v1700__uac_BarringFactorForAI3_r17_p90	= 14,
	UAC_BarringInfoSet_v1700__uac_BarringFactorForAI3_r17_p95	= 15
} e_UAC_BarringInfoSet_v1700__uac_BarringFactorForAI3_r17;

/* UAC-BarringInfoSet-v1700 */
typedef struct UAC_BarringInfoSet_v1700 {
	long	*uac_BarringFactorForAI3_r17;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} UAC_BarringInfoSet_v1700_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_uac_BarringFactorForAI3_r17_2;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_UAC_BarringInfoSet_v1700;
extern asn_SEQUENCE_specifics_t asn_SPC_UAC_BarringInfoSet_v1700_specs_1;
extern asn_TYPE_member_t asn_MBR_UAC_BarringInfoSet_v1700_1[1];

#ifdef __cplusplus
}
#endif

#endif	/* _UAC_BarringInfoSet_v1700_H_ */
#include <asn_internal.h>
