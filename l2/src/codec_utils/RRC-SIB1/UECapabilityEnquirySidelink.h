/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "PC5-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_UECapabilityEnquirySidelink_H_
#define	_UECapabilityEnquirySidelink_H_


#include <asn_application.h>

/* Including external dependencies */
#include "RRC-TransactionIdentifier.h"
#include <constr_SEQUENCE.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum UECapabilityEnquirySidelink__criticalExtensions_PR {
	UECapabilityEnquirySidelink__criticalExtensions_PR_NOTHING,	/* No components present */
	UECapabilityEnquirySidelink__criticalExtensions_PR_ueCapabilityEnquirySidelink_r16,
	UECapabilityEnquirySidelink__criticalExtensions_PR_criticalExtensionsFuture
} UECapabilityEnquirySidelink__criticalExtensions_PR;

/* Forward declarations */
struct UECapabilityEnquirySidelink_r16_IEs;

/* UECapabilityEnquirySidelink */
typedef struct UECapabilityEnquirySidelink {
	RRC_TransactionIdentifier_t	 rrc_TransactionIdentifier_r16;
	struct UECapabilityEnquirySidelink__criticalExtensions {
		UECapabilityEnquirySidelink__criticalExtensions_PR present;
		union UECapabilityEnquirySidelink__criticalExtensions_u {
			struct UECapabilityEnquirySidelink_r16_IEs	*ueCapabilityEnquirySidelink_r16;
			struct UECapabilityEnquirySidelink__criticalExtensions__criticalExtensionsFuture {
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *criticalExtensionsFuture;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} criticalExtensions;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} UECapabilityEnquirySidelink_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_UECapabilityEnquirySidelink;
extern asn_SEQUENCE_specifics_t asn_SPC_UECapabilityEnquirySidelink_specs_1;
extern asn_TYPE_member_t asn_MBR_UECapabilityEnquirySidelink_1[2];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "UECapabilityEnquirySidelink-r16-IEs.h"

#endif	/* _UECapabilityEnquirySidelink_H_ */
#include <asn_internal.h>
