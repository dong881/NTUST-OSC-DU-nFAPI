/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_CG_StartingOffsets_r16_H_
#define	_CG_StartingOffsets_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CG-StartingOffsets-r16 */
typedef struct CG_StartingOffsets_r16 {
	struct CG_StartingOffsets_r16__cg_StartingFullBW_InsideCOT_r16 {
		A_SEQUENCE_OF(long) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *cg_StartingFullBW_InsideCOT_r16;
	struct CG_StartingOffsets_r16__cg_StartingFullBW_OutsideCOT_r16 {
		A_SEQUENCE_OF(long) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *cg_StartingFullBW_OutsideCOT_r16;
	long	*cg_StartingPartialBW_InsideCOT_r16;	/* OPTIONAL */
	long	*cg_StartingPartialBW_OutsideCOT_r16;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CG_StartingOffsets_r16_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CG_StartingOffsets_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_CG_StartingOffsets_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_CG_StartingOffsets_r16_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _CG_StartingOffsets_r16_H_ */
#include <asn_internal.h>
