/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SimulSRS_ForAntennaSwitching_r16_H_
#define	_SimulSRS_ForAntennaSwitching_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SimulSRS_ForAntennaSwitching_r16__supportSRS_xTyR_xLessThanY_r16 {
	SimulSRS_ForAntennaSwitching_r16__supportSRS_xTyR_xLessThanY_r16_supported	= 0
} e_SimulSRS_ForAntennaSwitching_r16__supportSRS_xTyR_xLessThanY_r16;
typedef enum SimulSRS_ForAntennaSwitching_r16__supportSRS_xTyR_xEqualToY_r16 {
	SimulSRS_ForAntennaSwitching_r16__supportSRS_xTyR_xEqualToY_r16_supported	= 0
} e_SimulSRS_ForAntennaSwitching_r16__supportSRS_xTyR_xEqualToY_r16;
typedef enum SimulSRS_ForAntennaSwitching_r16__supportSRS_AntennaSwitching_r16 {
	SimulSRS_ForAntennaSwitching_r16__supportSRS_AntennaSwitching_r16_supported	= 0
} e_SimulSRS_ForAntennaSwitching_r16__supportSRS_AntennaSwitching_r16;

/* SimulSRS-ForAntennaSwitching-r16 */
typedef struct SimulSRS_ForAntennaSwitching_r16 {
	long	*supportSRS_xTyR_xLessThanY_r16;	/* OPTIONAL */
	long	*supportSRS_xTyR_xEqualToY_r16;	/* OPTIONAL */
	long	*supportSRS_AntennaSwitching_r16;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SimulSRS_ForAntennaSwitching_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_supportSRS_xTyR_xLessThanY_r16_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_supportSRS_xTyR_xEqualToY_r16_4;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_supportSRS_AntennaSwitching_r16_6;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SimulSRS_ForAntennaSwitching_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_SimulSRS_ForAntennaSwitching_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_SimulSRS_ForAntennaSwitching_r16_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _SimulSRS_ForAntennaSwitching_r16_H_ */
#include <asn_internal.h>
