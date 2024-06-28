/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_ApplicableDisasterInfo_r17_H_
#define	_ApplicableDisasterInfo_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NULL.h>
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ApplicableDisasterInfo_r17_PR {
	ApplicableDisasterInfo_r17_PR_NOTHING,	/* No components present */
	ApplicableDisasterInfo_r17_PR_noDisasterRoaming_r17,
	ApplicableDisasterInfo_r17_PR_disasterRelatedIndication_r17,
	ApplicableDisasterInfo_r17_PR_commonPLMNs_r17,
	ApplicableDisasterInfo_r17_PR_dedicatedPLMNs_r17
} ApplicableDisasterInfo_r17_PR;

/* Forward declarations */
struct PLMN_Identity;

/* ApplicableDisasterInfo-r17 */
typedef struct ApplicableDisasterInfo_r17 {
	ApplicableDisasterInfo_r17_PR present;
	union ApplicableDisasterInfo_r17_u {
		NULL_t	 noDisasterRoaming_r17;
		NULL_t	 disasterRelatedIndication_r17;
		NULL_t	 commonPLMNs_r17;
		struct ApplicableDisasterInfo_r17__dedicatedPLMNs_r17 {
			A_SEQUENCE_OF(struct PLMN_Identity) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *dedicatedPLMNs_r17;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ApplicableDisasterInfo_r17_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ApplicableDisasterInfo_r17;
extern asn_CHOICE_specifics_t asn_SPC_ApplicableDisasterInfo_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_ApplicableDisasterInfo_r17_1[4];
extern asn_per_constraints_t asn_PER_type_ApplicableDisasterInfo_r17_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "PLMN-Identity.h"

#endif	/* _ApplicableDisasterInfo_r17_H_ */
#include <asn_internal.h>
