/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_IAB_IP_Address_r16_H_
#define	_IAB_IP_Address_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <BIT_STRING.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum IAB_IP_Address_r16_PR {
	IAB_IP_Address_r16_PR_NOTHING,	/* No components present */
	IAB_IP_Address_r16_PR_iPv4_Address_r16,
	IAB_IP_Address_r16_PR_iPv6_Address_r16,
	IAB_IP_Address_r16_PR_iPv6_Prefix_r16
	/* Extensions may appear below */
	
} IAB_IP_Address_r16_PR;

/* IAB-IP-Address-r16 */
typedef struct IAB_IP_Address_r16 {
	IAB_IP_Address_r16_PR present;
	union IAB_IP_Address_r16_u {
		BIT_STRING_t	 iPv4_Address_r16;
		BIT_STRING_t	 iPv6_Address_r16;
		BIT_STRING_t	 iPv6_Prefix_r16;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} IAB_IP_Address_r16_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_IAB_IP_Address_r16;
extern asn_CHOICE_specifics_t asn_SPC_IAB_IP_Address_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_IAB_IP_Address_r16_1[3];
extern asn_per_constraints_t asn_PER_type_IAB_IP_Address_r16_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _IAB_IP_Address_r16_H_ */
#include <asn_internal.h>
