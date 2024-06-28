/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MAC_MainConfigSL_r16_H_
#define	_MAC_MainConfigSL_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct BSR_Config;

/* MAC-MainConfigSL-r16 */
typedef struct MAC_MainConfigSL_r16 {
	struct BSR_Config	*sl_BSR_Config_r16;	/* OPTIONAL */
	long	*ul_PrioritizationThres_r16;	/* OPTIONAL */
	long	*sl_PrioritizationThres_r16;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MAC_MainConfigSL_r16_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MAC_MainConfigSL_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_MAC_MainConfigSL_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_MAC_MainConfigSL_r16_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "BSR-Config.h"

#endif	/* _MAC_MainConfigSL_r16_H_ */
#include <asn_internal.h>
