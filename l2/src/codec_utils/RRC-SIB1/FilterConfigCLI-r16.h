/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_FilterConfigCLI_r16_H_
#define	_FilterConfigCLI_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include "FilterCoefficient.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* FilterConfigCLI-r16 */
typedef struct FilterConfigCLI_r16 {
	FilterCoefficient_t	*filterCoefficientSRS_RSRP_r16;	/* DEFAULT 4 */
	FilterCoefficient_t	*filterCoefficientCLI_RSSI_r16;	/* DEFAULT 4 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} FilterConfigCLI_r16_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_FilterConfigCLI_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_FilterConfigCLI_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_FilterConfigCLI_r16_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _FilterConfigCLI_r16_H_ */
#include <asn_internal.h>
