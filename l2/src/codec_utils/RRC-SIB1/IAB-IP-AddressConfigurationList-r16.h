/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_IAB_IP_AddressConfigurationList_r16_H_
#define	_IAB_IP_AddressConfigurationList_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include "IAB-IP-AddressIndex-r16.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct IAB_IP_AddressConfiguration_r16;

/* IAB-IP-AddressConfigurationList-r16 */
typedef struct IAB_IP_AddressConfigurationList_r16 {
	struct IAB_IP_AddressConfigurationList_r16__iab_IP_AddressToAddModList_r16 {
		A_SEQUENCE_OF(struct IAB_IP_AddressConfiguration_r16) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *iab_IP_AddressToAddModList_r16;
	struct IAB_IP_AddressConfigurationList_r16__iab_IP_AddressToReleaseList_r16 {
		A_SEQUENCE_OF(IAB_IP_AddressIndex_r16_t) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *iab_IP_AddressToReleaseList_r16;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} IAB_IP_AddressConfigurationList_r16_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_IAB_IP_AddressConfigurationList_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_IAB_IP_AddressConfigurationList_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_IAB_IP_AddressConfigurationList_r16_1[2];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "IAB-IP-AddressConfiguration-r16.h"

#endif	/* _IAB_IP_AddressConfigurationList_r16_H_ */
#include <asn_internal.h>
