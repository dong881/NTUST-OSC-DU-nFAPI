/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_BeamFailureDetection_r17_H_
#define	_BeamFailureDetection_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include "AdditionalPCIIndex-r17.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct BeamFailureDetectionSet_r17;

/* BeamFailureDetection-r17 */
typedef struct BeamFailureDetection_r17 {
	struct BeamFailureDetectionSet_r17	*failureDetectionSet1_r17;	/* OPTIONAL */
	struct BeamFailureDetectionSet_r17	*failureDetectionSet2_r17;	/* OPTIONAL */
	AdditionalPCIIndex_r17_t	*additionalPCI_r17;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} BeamFailureDetection_r17_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_BeamFailureDetection_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_BeamFailureDetection_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_BeamFailureDetection_r17_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "BeamFailureDetectionSet-r17.h"

#endif	/* _BeamFailureDetection_r17_H_ */
#include <asn_internal.h>
