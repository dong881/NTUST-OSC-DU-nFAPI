/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_BAP_Parameters_v1700_H_
#define	_BAP_Parameters_v1700_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum BAP_Parameters_v1700__bapHeaderRewriting_Rerouting_r17 {
	BAP_Parameters_v1700__bapHeaderRewriting_Rerouting_r17_supported	= 0
} e_BAP_Parameters_v1700__bapHeaderRewriting_Rerouting_r17;
typedef enum BAP_Parameters_v1700__bapHeaderRewriting_Routing_r17 {
	BAP_Parameters_v1700__bapHeaderRewriting_Routing_r17_supported	= 0
} e_BAP_Parameters_v1700__bapHeaderRewriting_Routing_r17;

/* BAP-Parameters-v1700 */
typedef struct BAP_Parameters_v1700 {
	long	*bapHeaderRewriting_Rerouting_r17;	/* OPTIONAL */
	long	*bapHeaderRewriting_Routing_r17;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} BAP_Parameters_v1700_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_bapHeaderRewriting_Rerouting_r17_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_bapHeaderRewriting_Routing_r17_4;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_BAP_Parameters_v1700;
extern asn_SEQUENCE_specifics_t asn_SPC_BAP_Parameters_v1700_specs_1;
extern asn_TYPE_member_t asn_MBR_BAP_Parameters_v1700_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _BAP_Parameters_v1700_H_ */
#include <asn_internal.h>
