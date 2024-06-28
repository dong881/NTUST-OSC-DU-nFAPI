/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MBSInterestIndication_r17_H_
#define	_MBSInterestIndication_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include <constr_SEQUENCE.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum MBSInterestIndication_r17__criticalExtensions_PR {
	MBSInterestIndication_r17__criticalExtensions_PR_NOTHING,	/* No components present */
	MBSInterestIndication_r17__criticalExtensions_PR_mbsInterestIndication_r17,
	MBSInterestIndication_r17__criticalExtensions_PR_criticalExtensionsFuture
} MBSInterestIndication_r17__criticalExtensions_PR;

/* Forward declarations */
struct MBSInterestIndication_r17_IEs;

/* MBSInterestIndication-r17 */
typedef struct MBSInterestIndication_r17 {
	struct MBSInterestIndication_r17__criticalExtensions {
		MBSInterestIndication_r17__criticalExtensions_PR present;
		union MBSInterestIndication_r17__criticalExtensions_u {
			struct MBSInterestIndication_r17_IEs	*mbsInterestIndication_r17;
			struct MBSInterestIndication_r17__criticalExtensions__criticalExtensionsFuture {
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *criticalExtensionsFuture;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} criticalExtensions;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MBSInterestIndication_r17_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MBSInterestIndication_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_MBSInterestIndication_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_MBSInterestIndication_r17_1[1];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "MBSInterestIndication-r17-IEs.h"

#endif	/* _MBSInterestIndication_r17_H_ */
#include <asn_internal.h>
