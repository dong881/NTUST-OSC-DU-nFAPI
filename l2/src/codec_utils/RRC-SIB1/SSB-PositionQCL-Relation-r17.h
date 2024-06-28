/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SSB_PositionQCL_Relation_r17_H_
#define	_SSB_PositionQCL_Relation_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SSB_PositionQCL_Relation_r17 {
	SSB_PositionQCL_Relation_r17_n32	= 0,
	SSB_PositionQCL_Relation_r17_n64	= 1
} e_SSB_PositionQCL_Relation_r17;

/* SSB-PositionQCL-Relation-r17 */
typedef long	 SSB_PositionQCL_Relation_r17_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_SSB_PositionQCL_Relation_r17_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_SSB_PositionQCL_Relation_r17;
extern const asn_INTEGER_specifics_t asn_SPC_SSB_PositionQCL_Relation_r17_specs_1;
asn_struct_free_f SSB_PositionQCL_Relation_r17_free;
asn_struct_print_f SSB_PositionQCL_Relation_r17_print;
asn_constr_check_f SSB_PositionQCL_Relation_r17_constraint;
xer_type_decoder_f SSB_PositionQCL_Relation_r17_decode_xer;
xer_type_encoder_f SSB_PositionQCL_Relation_r17_encode_xer;
per_type_decoder_f SSB_PositionQCL_Relation_r17_decode_uper;
per_type_encoder_f SSB_PositionQCL_Relation_r17_encode_uper;
per_type_decoder_f SSB_PositionQCL_Relation_r17_decode_aper;
per_type_encoder_f SSB_PositionQCL_Relation_r17_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _SSB_PositionQCL_Relation_r17_H_ */
#include <asn_internal.h>
