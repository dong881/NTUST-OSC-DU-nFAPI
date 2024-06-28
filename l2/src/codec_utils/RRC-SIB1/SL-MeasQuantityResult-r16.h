/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "PC5-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SL_MeasQuantityResult_r16_H_
#define	_SL_MeasQuantityResult_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include "RSRP-Range.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SL-MeasQuantityResult-r16 */
typedef struct SL_MeasQuantityResult_r16 {
	RSRP_Range_t	*sl_RSRP_r16;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SL_MeasQuantityResult_r16_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SL_MeasQuantityResult_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_SL_MeasQuantityResult_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_SL_MeasQuantityResult_r16_1[1];

#ifdef __cplusplus
}
#endif

#endif	/* _SL_MeasQuantityResult_r16_H_ */
#include <asn_internal.h>
