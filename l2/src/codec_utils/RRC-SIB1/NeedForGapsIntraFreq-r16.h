/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_NeedForGapsIntraFreq_r16_H_
#define	_NeedForGapsIntraFreq_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include "ServCellIndex.h"
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum NeedForGapsIntraFreq_r16__gapIndicationIntra_r16 {
	NeedForGapsIntraFreq_r16__gapIndicationIntra_r16_gap	= 0,
	NeedForGapsIntraFreq_r16__gapIndicationIntra_r16_no_gap	= 1
} e_NeedForGapsIntraFreq_r16__gapIndicationIntra_r16;

/* NeedForGapsIntraFreq-r16 */
typedef struct NeedForGapsIntraFreq_r16 {
	ServCellIndex_t	 servCellId_r16;
	long	 gapIndicationIntra_r16;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} NeedForGapsIntraFreq_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_gapIndicationIntra_r16_3;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_NeedForGapsIntraFreq_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_NeedForGapsIntraFreq_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_NeedForGapsIntraFreq_r16_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _NeedForGapsIntraFreq_r16_H_ */
#include <asn_internal.h>
