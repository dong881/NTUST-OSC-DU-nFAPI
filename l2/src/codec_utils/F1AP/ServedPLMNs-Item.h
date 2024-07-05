/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "F1AP-IEs"
 * 	found in "../F1.asn1"
 * 	`asn1c -D ../F1_output/ -fcompound-names -fno-include-deps -findirect-choice -gen-PER`
 */

#ifndef	_ServedPLMNs_Item_H_
#define	_ServedPLMNs_Item_H_


#include <asn_application.h>

/* Including external dependencies */
#include "PLMN-IdentityF1AP.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ProtocolExtensionContainer;

/* ServedPLMNs-Item */
typedef struct ServedPLMNs_Item {
	PLMN_IdentityF1AP_t	 pLMN_Identity;
	struct ProtocolExtensionContainer_4624P3	*iE_Extensions	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ServedPLMNs_Item_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ServedPLMNs_Item;
extern asn_SEQUENCE_specifics_t asn_SPC_ServedPLMNs_Item_specs_1;
extern asn_TYPE_member_t asn_MBR_ServedPLMNs_Item_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _ServedPLMNs_Item_H_ */
#include <asn_internal.h>
