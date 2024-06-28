/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_T_StatusProhibit_v1610_H_
#define	_T_StatusProhibit_v1610_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum T_StatusProhibit_v1610 {
	T_StatusProhibit_v1610_ms1	= 0,
	T_StatusProhibit_v1610_ms2	= 1,
	T_StatusProhibit_v1610_ms3	= 2,
	T_StatusProhibit_v1610_ms4	= 3,
	T_StatusProhibit_v1610_spare4	= 4,
	T_StatusProhibit_v1610_spare3	= 5,
	T_StatusProhibit_v1610_spare2	= 6,
	T_StatusProhibit_v1610_spare1	= 7
} e_T_StatusProhibit_v1610;

/* T-StatusProhibit-v1610 */
typedef long	 T_StatusProhibit_v1610_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_T_StatusProhibit_v1610_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_T_StatusProhibit_v1610;
extern const asn_INTEGER_specifics_t asn_SPC_T_StatusProhibit_v1610_specs_1;
asn_struct_free_f T_StatusProhibit_v1610_free;
asn_struct_print_f T_StatusProhibit_v1610_print;
asn_constr_check_f T_StatusProhibit_v1610_constraint;
xer_type_decoder_f T_StatusProhibit_v1610_decode_xer;
xer_type_encoder_f T_StatusProhibit_v1610_encode_xer;
per_type_decoder_f T_StatusProhibit_v1610_decode_uper;
per_type_encoder_f T_StatusProhibit_v1610_encode_uper;
per_type_decoder_f T_StatusProhibit_v1610_decode_aper;
per_type_encoder_f T_StatusProhibit_v1610_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _T_StatusProhibit_v1610_H_ */
#include <asn_internal.h>
