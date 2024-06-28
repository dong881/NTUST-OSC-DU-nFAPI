/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SidelinkUEInformationNR_r16_H_
#define	_SidelinkUEInformationNR_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <constr_SEQUENCE.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SidelinkUEInformationNR_r16__criticalExtensions_PR {
	SidelinkUEInformationNR_r16__criticalExtensions_PR_NOTHING,	/* No components present */
	SidelinkUEInformationNR_r16__criticalExtensions_PR_sidelinkUEInformationNR_r16,
	SidelinkUEInformationNR_r16__criticalExtensions_PR_criticalExtensionsFuture
} SidelinkUEInformationNR_r16__criticalExtensions_PR;

/* Forward declarations */
struct SidelinkUEInformationNR_r16_IEs;

/* SidelinkUEInformationNR-r16 */
typedef struct SidelinkUEInformationNR_r16 {
	struct SidelinkUEInformationNR_r16__criticalExtensions {
		SidelinkUEInformationNR_r16__criticalExtensions_PR present;
		union SidelinkUEInformationNR_r16__criticalExtensions_u {
			struct SidelinkUEInformationNR_r16_IEs	*sidelinkUEInformationNR_r16;
			struct SidelinkUEInformationNR_r16__criticalExtensions__criticalExtensionsFuture {
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *criticalExtensionsFuture;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} criticalExtensions;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SidelinkUEInformationNR_r16_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SidelinkUEInformationNR_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_SidelinkUEInformationNR_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_SidelinkUEInformationNR_r16_1[1];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "SidelinkUEInformationNR-r16-IEs.h"

#endif	/* _SidelinkUEInformationNR_r16_H_ */
#include <asn_internal.h>
