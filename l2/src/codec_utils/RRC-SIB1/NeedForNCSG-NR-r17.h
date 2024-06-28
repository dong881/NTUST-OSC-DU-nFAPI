/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_NeedForNCSG_NR_r17_H_
#define	_NeedForNCSG_NR_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include "FreqBandIndicatorNR.h"
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum NeedForNCSG_NR_r17__gapIndication_r17 {
	NeedForNCSG_NR_r17__gapIndication_r17_gap	= 0,
	NeedForNCSG_NR_r17__gapIndication_r17_ncsg	= 1,
	NeedForNCSG_NR_r17__gapIndication_r17_nogap_noncsg	= 2
} e_NeedForNCSG_NR_r17__gapIndication_r17;

/* NeedForNCSG-NR-r17 */
typedef struct NeedForNCSG_NR_r17 {
	FreqBandIndicatorNR_t	 bandNR_r17;
	long	 gapIndication_r17;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} NeedForNCSG_NR_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_gapIndication_r17_3;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_NeedForNCSG_NR_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_NeedForNCSG_NR_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_NeedForNCSG_NR_r17_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _NeedForNCSG_NR_r17_H_ */
#include <asn_internal.h>
