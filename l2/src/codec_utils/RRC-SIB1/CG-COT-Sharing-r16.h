/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_CG_COT_Sharing_r16_H_
#define	_CG_COT_Sharing_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NULL.h>
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CG_COT_Sharing_r16_PR {
	CG_COT_Sharing_r16_PR_NOTHING,	/* No components present */
	CG_COT_Sharing_r16_PR_noCOT_Sharing_r16,
	CG_COT_Sharing_r16_PR_cot_Sharing_r16
} CG_COT_Sharing_r16_PR;

/* CG-COT-Sharing-r16 */
typedef struct CG_COT_Sharing_r16 {
	CG_COT_Sharing_r16_PR present;
	union CG_COT_Sharing_r16_u {
		NULL_t	 noCOT_Sharing_r16;
		struct CG_COT_Sharing_r16__cot_Sharing_r16 {
			long	 duration_r16;
			long	 offset_r16;
			long	 channelAccessPriority_r16;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *cot_Sharing_r16;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CG_COT_Sharing_r16_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CG_COT_Sharing_r16;
extern asn_CHOICE_specifics_t asn_SPC_CG_COT_Sharing_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_CG_COT_Sharing_r16_1[2];
extern asn_per_constraints_t asn_PER_type_CG_COT_Sharing_r16_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _CG_COT_Sharing_r16_H_ */
#include <asn_internal.h>
