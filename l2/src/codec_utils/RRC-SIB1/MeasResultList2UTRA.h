/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MeasResultList2UTRA_H_
#define	_MeasResultList2UTRA_H_


#include <asn_application.h>

/* Including external dependencies */
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct MeasResult2UTRA_FDD_r16;

/* MeasResultList2UTRA */
typedef struct MeasResultList2UTRA {
	A_SEQUENCE_OF(struct MeasResult2UTRA_FDD_r16) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MeasResultList2UTRA_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MeasResultList2UTRA;
extern asn_SET_OF_specifics_t asn_SPC_MeasResultList2UTRA_specs_1;
extern asn_TYPE_member_t asn_MBR_MeasResultList2UTRA_1[1];
extern asn_per_constraints_t asn_PER_type_MeasResultList2UTRA_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "MeasResult2UTRA-FDD-r16.h"

#endif	/* _MeasResultList2UTRA_H_ */
#include <asn_internal.h>
