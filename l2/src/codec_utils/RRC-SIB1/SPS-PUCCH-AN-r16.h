/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SPS_PUCCH_AN_r16_H_
#define	_SPS_PUCCH_AN_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include "PUCCH-ResourceId.h"
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SPS-PUCCH-AN-r16 */
typedef struct SPS_PUCCH_AN_r16 {
	PUCCH_ResourceId_t	 sps_PUCCH_AN_ResourceID_r16;
	long	*maxPayloadSize_r16;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SPS_PUCCH_AN_r16_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SPS_PUCCH_AN_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_SPS_PUCCH_AN_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_SPS_PUCCH_AN_r16_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _SPS_PUCCH_AN_r16_H_ */
#include <asn_internal.h>
